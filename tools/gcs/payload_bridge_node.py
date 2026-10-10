#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/gcs/payload_bridge_node.py — 投放执行桥（真机包 2026-10-09）。

RFL:88-92 立案缺口落地：mission_executor 在 AT_DROP 放行后发
/drone_<id>/payload/command（UInt8，TYPE_A..E→0..4，CFG type_to_uint8），
等 /drone_<id>/payload/done（Bool）才转 RETURN——sim 由 drop_simulator
0.5s 假应答，真机缺桥 = executor 永久停在 DROP（安全侧失败：不误投，
但也不返航）。本桥是机构的 ROS 侧接口：

  backend auto   机构未接的地面联调档——delay 秒后直接回 done=True
                 （drop_simulator 同语义）。【真机带弹飞行严禁 auto：
                 假投放假完成，机会带着弹箱「已投」返航】
  backend serial CLI 串口档——发 ASCII 行 "DROP <u8>\\n"，读应答行：
                 "OK" → done=True；超时 → done=False（executor 无 DROP
                 看门狗 = 停在 DROP 安全侧，不误返航）。
  应答行协议属机构控制器固件，联调日对质后只改 _serialize/_parse
  两个钩子（字节序/多行握手不进主流程）。

诚实边界：
  * 每条 command 至多一条 done；executor 每 MATCH 只发一次命令不重发
    （EX:513 定谳），本桥不补发不重试——重试语义属机构/人工层。
  * 串口档 I/O 放线程（执行器回调链禁止阻塞）；command 去抖 1s
    （同一 u8 重复命令忽略，机构续触发安全）。
  * 无 done 超时兜底是有意为之（安全侧），别在本桥「补齐」它。

用法（机载）：
  python3 payload_bridge_node.py --id 0 --backend auto
  python3 payload_bridge_node.py --id 0 --backend serial --port /dev/ttyUSB0
  离线：python3 payload_bridge_node.py --selftest
"""
import argparse
import sys
import threading
import time


# ---------------------------------------------------------------- 纯逻辑 --
def serialize_cmd(u8):
    """行协议下行帧（联调日对质固件后如需改，只改这里）。"""
    return ("DROP %d\n" % int(u8)).encode("ascii")


def parse_ack(line):
    """应答行 → True(OK)/False(明确拒绝)/None(非应答，继续等)。"""
    s = line.strip().upper()
    if s.startswith("OK"):
        return True
    if s.startswith(("ERR", "NOK", "BUSY", "REJECT")):
        return False
    return None


# ---------------------------------------------------------------- ROS 壳 --
class PayloadBridge(object):
    def __init__(self, args):
        import rospy
        from std_msgs.msg import UInt8, Bool, String
        self._rospy = rospy
        rospy.init_node("payload_bridge_node", anonymous=True)
        self.ns = "/drone_%d" % args.id
        self.backend = args.backend
        self.delay = args.delay
        self.ack_timeout = args.ack_timeout
        self.debounce = args.debounce
        self._last_cmd = None
        self._last_t = 0.0
        self._ser = None
        if self.backend == "serial":
            import serial  # pyserial：仅 serial 档需要
            self._ser = serial.Serial(args.port, args.baud, timeout=0.2)
            rospy.loginfo("payload_bridge: serial %s@%d open", args.port,
                          args.baud)
        self.pub_done = rospy.Publisher(self.ns + "/payload/done", Bool,
                                        queue_size=1)
        self.pub_ack = rospy.Publisher(self.ns + "/payload/ack", String,
                                       queue_size=1, latch=True)
        rospy.Subscriber(self.ns + "/payload/command", UInt8,
                         self.on_command, queue_size=2)
        rospy.loginfo("payload_bridge: drone %d backend=%s%s",
                      args.id, self.backend,
                      (" port=" + args.port) if args.port else "")
        rospy.spin()

    def on_command(self, m):
        now = time.time()
        u8 = int(m.data)
        if u8 == self._last_cmd and now - self._last_t < self.debounce:
            return
        self._last_cmd, self._last_t = u8, now
        t = threading.Thread(target=self._execute, args=(u8,), daemon=True)
        t.start()

    def _execute(self, u8):
        r = self._rospy
        if self.backend == "auto":
            r.loginfo("payload_bridge: auto backend dropping (u8=%d) — "
                      "GROUND TEST ONLY", u8)
            time.sleep(self.delay)
            self.pub_done.publish(Bool(data=True))
            self.pub_ack.publish(self._String(data="AUTO_DONE u8=%d" % u8))
            return
        # serial 档
        try:
            self._ser.reset_input_buffer()
            self._ser.write(serialize_cmd(u8))
            self._ser.flush()
        except Exception as e:
            r.logerr("payload_bridge: serial write failed: %s", e)
            self.pub_done.publish(Bool(data=False))
            self.pub_ack.publish(self._String(data="WRITE_FAIL %s" % e))
            return
        t0 = time.time()
        buf = b""
        while time.time() - t0 < self.ack_timeout and not r.is_shutdown():
            try:
                buf += self._ser.read(64)
            except Exception as e:
                r.logerr("payload_bridge: serial read failed: %s", e)
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                ack = parse_ack(line.decode("ascii", "ignore"))
                if ack is None:
                    continue
                self.pub_done.publish(Bool(data=bool(ack)))
                self.pub_ack.publish(self._String(
                    data=("ACK u8=%d" % u8) if ack else
                    ("REJECT u8=%d %s" % (u8, line.strip().decode()[:40]))))
                if ack:
                    r.loginfo("payload_bridge: drop u8=%d acked", u8)
                else:
                    r.logwarn("payload_bridge: drop u8=%d rejected by "
                              "actuator (done=False, executor stays in DROP)",
                              u8)
                return
        r.logwarn("payload_bridge: drop u8=%d ACK timeout %.1fs -> done=False"
                  " (executor stays in DROP)", u8, self.ack_timeout)
        self.pub_done.publish(Bool(data=False))
        self.pub_ack.publish(self._String(data="TIMEOUT u8=%d" % u8))

    @property
    def _String(self):
        from std_msgs.msg import String
        return String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--backend", default="auto", choices=("auto", "serial"))
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="serial 档串口设备")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--delay", type=float, default=0.5,
                    help="auto 档假应答延迟（drop_simulator 同款 0.5s）")
    ap.add_argument("--ack-timeout", type=float, default=3.0)
    ap.add_argument("--debounce", type=float, default=1.0)
    PayloadBridge(ap.parse_args())


def _selftest():
    """纯逻辑离线自检（--selftest）：行协议编解码。"""
    assert serialize_cmd(0) == b"DROP 0\n"
    assert serialize_cmd(4) == b"DROP 4\n"
    assert parse_ack("OK\n") is True
    assert parse_ack("ok done") is True
    assert parse_ack("ERR pin") is False
    assert parse_ack("BUSY") is False
    assert parse_ack("DROP 3") is None      # 回显非应答
    assert parse_ack("") is None
    assert parse_ack("  REJECT no-mag") is False
    print("PAYLOAD_BRIDGE SELFTEST PASS")


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        _selftest()
    else:
        main()
