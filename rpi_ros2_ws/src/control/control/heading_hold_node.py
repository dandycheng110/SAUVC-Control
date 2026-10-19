"""航向保持：操作者放開轉向鍵之後，載具維持在放開時的方向。

為什麼需要：載具在水中轉向幾乎沒有阻尼。模擬實測在 0.5 m 深給 +12 N·m，
兩秒轉了三百多度；放手的瞬間它還在用每秒 150° 以上的速度轉，開迴路的
鍵盤轉向只能「轉過頭再反打」，看起來就是笨。

行為照 ArduSub 的 Stabilize／Depth Hold：操作者給的是**轉速**，不是力矩。
  * control/targets/yaw_rate_dps：操作者要的轉速（GUI 在航向保持開著時，
    左右鍵送這個，不再送 torque.z）。目標航向以這個速度往前推，PID 追它，
    同時把轉速當前饋，所以轉起來的速度固定，跟載具阻尼無關。
  * 放手（轉速歸零）時目標當場停住，PID 把載具煞在那裡 —— 不會像純力矩
    那樣甩出去幾十度再拉回來。
  * control/targets/heading_deg：直接指定目標（GUI 的 [ ] 鍵；之後自主端
    也可以用）。

後備：如果有人在航向保持開著時直接送 GUI torque.z（例如 Tuning 頁的手動
wrench），本節點讓手、輸出零，放開 pilot_release_delay_s 之後再抓當下航向。

航向的定義：從上往下看順時針為正、單位度、範圍 0～360。這是機體
forward-right-down 座標下 +torque.z 的轉向，所以誤差、力矩與顯示同一個
正方向。0° 是 IMU 姿態參考系的 x 軸 —— 模擬裡是開機時的朝向，不是北方。

微分項用「量到的轉速」而不是誤差的變化量：目標被 [ ] 鍵一次跳 15° 時，
誤差微分會在一個週期內暴衝（derivative kick），量測值的微分不會。
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Wrench
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


# 兩筆姿態間隔短於這個值就不拿來算轉速。executor 有時會連續處理兩筆排隊的
# 訊息，間隔只有零點幾毫秒，角度差卻是 10 ms 的量，算出來的轉速會爆掉。
_MIN_RATE_SAMPLE_INTERVAL_S = 0.005
# 比這個小的轉速指令當作零（度/秒）。
_RATE_COMMAND_DEADBAND_DPS = 0.1


def wrap_to_pi(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def heading_from_quaternion(w: float, x: float, y: float, z: float) -> float:
    """機體 x 軸的航向（弧度，從上往下看順時針為正）。

    姿態的參考系上游沒有統一：模擬的 IMU 回報的是相對於開機姿態（z 朝下），
    實機的 IMU 可能是 ENU（z 朝上）。z 朝上的參考系裡 atan2 是逆時針為正，
    所以正方向由機體 z 軸（朝下）落在參考系的哪一側決定，兩種都能用。
    """
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_y = 2.0 * (x * y + w * z)
    down_z = 1.0 - 2.0 * (x * x + y * y)
    sense = 1.0 if down_z >= 0.0 else -1.0
    return sense * math.atan2(forward_y, forward_x)


class HeadingHoldNode(LifecycleNode):

    def __init__(self):
        super().__init__('heading_hold_node')

        # 熱調。名稱與意義和 generic_pid_controller_node 一致，GUI 的調參面板
        # 與 `ros2 param set` 用起來一樣。單位：P 是 N·m/rad，D 是 N·m/(rad/s)，
        # 兩個限幅是 N·m（<= 0 代表停用）。
        self.declare_parameter('proportional_gain', 0.0)
        self.declare_parameter('integral_gain', 0.0)
        self.declare_parameter('derivative_gain', 0.0)
        # 對量到的轉速做平滑，0 = 不平滑。公式與 generic_pid_controller_node 相同。
        self.declare_parameter('derivative_smoothing_factor', 0.0)
        self.declare_parameter('integral_limit', 0.0)
        self.declare_parameter('output_limit', 0.0)
        # 熱調。誤差小於這個角度才累積積分。大角度轉向時積分若照樣累積，
        # 到位之後會反過來把載具推離目標好幾秒（實測殘留 2～5°）。
        self.declare_parameter('integral_zone_deg', 10.0)

        # 熱調。轉速指令：目標最多領先實際航向多少。載具跟不上時（推力飽和），
        # 目標不能無限往前跑，否則放手後載具還要追很久才停。
        self.declare_parameter('max_target_lead_deg', 30.0)
        self.declare_parameter('rate_command_timeout_s', 0.3)

        # 熱調。後備路徑：GUI 直接送 torque.z 時的讓手規則。
        self.declare_parameter('pilot_yaw_deadband_Nm', 0.05)
        self.declare_parameter('pilot_release_delay_s', 0.3)
        # GUI 只在有按鍵時發 wrench，放開時補一筆零。超過這個時間沒收到，
        # 就當作操作者已經放手，免得瀏覽器斷線時本節點永遠讓手。
        self.declare_parameter('pilot_input_timeout_s', 0.3)
        self.declare_parameter('orientation_timeout_s', 0.5)

        # 開機讀
        self.declare_parameter('controller_loop_timer_period_s', 0.02)
        self.declare_parameter('status_period_s', 0.1)

        self._period_s = float(self.get_parameter('controller_loop_timer_period_s').value)
        status_period_s = float(self.get_parameter('status_period_s').value)

        self._active = False

        self._heading = None
        self._heading_stamp = None
        self._rate_reference = None
        self._rate = 0.0

        self._rate_command = 0.0
        self._rate_command_stamp = None

        self._pilot_torque_Nm = 0.0
        self._pilot_stamp = None
        self._pilot_last_active = None

        # 目標航向（弧度）。None 代表還沒抓：剛啟用、操作者正用力矩在轉、
        # 或姿態斷過。
        self._target = None
        self._integral = 0.0

        self.create_subscription(
            Quaternion, 'state/orientation', self._on_orientation, 10)
        self.create_subscription(
            Float64, 'control/targets/yaw_rate_dps', self._on_rate_command, 10)
        self.create_subscription(
            Float64, 'control/targets/heading_deg', self._on_target, 10)
        self.create_subscription(
            Wrench, 'control/wrench_sources/gui', self._on_pilot_wrench, 10)

        # 直接發到自己的匯流排來源，不像 depth 那樣發 control/wrench_command
        # 再靠 launch remap：那個名字是分配層的輸入，忘了 remap 就會繞過
        # wrench_sum 和 supervisor 的閘門直接推推進器。
        self._wrench_publisher = self.create_publisher(
            Wrench, 'control/wrench_sources/heading', 10)
        self._heading_publisher = self.create_publisher(Float64, 'state/heading_deg', 10)
        self._target_publisher = self.create_publisher(
            Float64, 'state/heading_target_deg', 10)

        self.create_service(Trigger, f'{self.get_name()}/reset', self._on_reset)
        self.create_timer(self._period_s, self._control_loop)
        self.create_timer(status_period_s, self._publish_status)
        self.add_on_set_parameters_callback(self._on_params)

    # ------------------------------------------------------------- lifecycle

    def on_configure(self, state) -> TransitionCallbackReturn:
        smoothing = float(self.get_parameter('derivative_smoothing_factor').value)
        if not self._is_valid_smoothing_factor(smoothing):
            self.get_logger().error(
                'derivative_smoothing_factor must be finite and between 0.0 and 1.0')
            return TransitionCallbackReturn.FAILURE
        self._reset_state()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        # 啟用的瞬間就抓當下的航向，不沿用任何舊目標：模式切換不該讓載具突然轉。
        self._reset_state()
        self._active = True
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        return self._stop()

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        return self._stop()

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        return self._stop()

    def _stop(self) -> TransitionCallbackReturn:
        self._active = False
        self._reset_state()
        self._publish_torque(0.0)
        return TransitionCallbackReturn.SUCCESS

    # ---------------------------------------------------------------- inputs

    def _on_orientation(self, msg: Quaternion):
        now = self.get_clock().now()
        heading = heading_from_quaternion(msg.w, msg.x, msg.y, msg.z)
        self._heading = heading
        self._heading_stamp = now

        if self._rate_reference is None:
            self._rate_reference = (heading, now)
            return
        previous_heading, previous_stamp = self._rate_reference
        interval_s = self._age_s(previous_stamp, now)
        if interval_s < _MIN_RATE_SAMPLE_INTERVAL_S:
            return
        if interval_s > 1.0:
            # 姿態斷過，舊的參考點沒有意義。
            self._rate_reference = (heading, now)
            self._rate = 0.0
            return

        raw_rate = wrap_to_pi(heading - previous_heading) / interval_s
        smoothing = float(self.get_parameter('derivative_smoothing_factor').value)
        retained = smoothing ** interval_s
        self._rate = (1.0 - retained) * raw_rate + retained * self._rate
        self._rate_reference = (heading, now)

    def _on_rate_command(self, msg: Float64):
        if not math.isfinite(msg.data):
            self.get_logger().warning(f'Ignoring non-finite yaw rate command: {msg.data}')
            return
        self._rate_command = math.radians(msg.data)
        self._rate_command_stamp = self.get_clock().now()

    def _on_pilot_wrench(self, msg: Wrench):
        self._pilot_torque_Nm = float(msg.torque.z)
        self._pilot_stamp = self.get_clock().now()

    def _on_target(self, msg: Float64):
        if not self._active:
            # 啟用時一律抓當下航向（見 on_activate），先存起來也用不到。
            return
        if not math.isfinite(msg.data):
            self.get_logger().warning(f'Ignoring non-finite heading target: {msg.data}')
            return
        self._target = wrap_to_pi(math.radians(msg.data))
        # 明確指定的目標立刻生效，不等後備路徑的放手延遲。
        self._pilot_last_active = None

    def _on_reset(self, request, response):
        self._reset_state()
        response.success = True
        response.message = 'Heading hold state reset'
        return response

    def _on_params(self, params):
        for param in params:
            if param.name != 'derivative_smoothing_factor':
                continue
            try:
                value = float(param.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason='derivative_smoothing_factor must be a number')
            if not self._is_valid_smoothing_factor(value):
                return SetParametersResult(
                    successful=False,
                    reason='derivative_smoothing_factor must be finite and between 0.0 and 1.0')
        return SetParametersResult(successful=True)

    # ----------------------------------------------------------------- loop

    def _control_loop(self):
        if not self._active:
            return
        now = self.get_clock().now()

        if not self._orientation_fresh(now):
            # 沒有姿態就守不住航向。輸出零，等姿態回來再重新抓目標 ——
            # 沿用舊目標的話，載具在斷線期間轉了多少，回來時就會猛轉回去。
            self._reset_state()
            self._publish_torque(0.0)
            return

        if self._pilot_torque_active(now):
            self._pilot_last_active = now
            self._target = None
            self._publish_torque(0.0)
            return

        rate_command = self._fresh_rate_command(now)

        if self._target is None:
            release_delay_s = float(self.get_parameter('pilot_release_delay_s').value)
            if (rate_command == 0.0
                    and self._pilot_last_active is not None
                    and self._age_s(self._pilot_last_active, now) < release_delay_s):
                self._publish_torque(0.0)
                return
            self._target = self._heading

        if rate_command != 0.0:
            self._target = wrap_to_pi(self._target + rate_command * self._period_s)
            max_lead = math.radians(float(self.get_parameter('max_target_lead_deg').value))
            lead = wrap_to_pi(self._target - self._heading)
            if max_lead > 0.0 and abs(lead) > max_lead:
                self._target = wrap_to_pi(self._heading + math.copysign(max_lead, lead))

        error = wrap_to_pi(self._target - self._heading)
        self._publish_torque(self._pid(error, rate_command))

    def _pid(self, error: float, rate_command: float) -> float:
        proportional_gain = float(self.get_parameter('proportional_gain').value)
        integral_gain = float(self.get_parameter('integral_gain').value)
        derivative_gain = float(self.get_parameter('derivative_gain').value)
        integral_limit = float(self.get_parameter('integral_limit').value)
        output_limit = float(self.get_parameter('output_limit').value)
        integral_zone = math.radians(float(self.get_parameter('integral_zone_deg').value))

        # 轉向中或離目標還遠時不累積：那時候的誤差不是「外力造成的偏差」，
        # 積分記住它只會在到位後變成反向推力。
        if rate_command == 0.0 and abs(error) < integral_zone:
            self._integral += error * self._period_s
        integral_output = integral_gain * self._integral
        if integral_limit > 0.0 and abs(integral_output) > integral_limit:
            integral_output = math.copysign(integral_limit, integral_output)
            # 同 generic_pid_controller_node：累加值倒算回邊界，不在背景繼續長。
            if integral_gain != 0.0:
                self._integral = integral_output / integral_gain

        # 微分項追「指令轉速 - 實際轉速」：放手時指令是零，就是純阻尼；
        # 轉向時指令轉速當前饋，不然載具會一直落後目標一截。
        torque = (proportional_gain * error
                  + integral_output
                  + derivative_gain * (rate_command - self._rate))
        if output_limit > 0.0 and abs(torque) > output_limit:
            torque = math.copysign(output_limit, torque)
        return torque

    # ---------------------------------------------------------------- output

    def _publish_torque(self, torque_Nm: float):
        msg = Wrench()
        msg.torque.z = float(torque_Nm)
        self._wrench_publisher.publish(msg)

    def _publish_status(self):
        # 不管有沒有啟用都發目前航向：GUI 在開航向保持之前就需要看得到。
        now = self.get_clock().now()
        heading_msg = Float64()
        heading_msg.data = (math.degrees(self._heading) % 360.0
                            if self._orientation_fresh(now) else math.nan)
        self._heading_publisher.publish(heading_msg)

        target_msg = Float64()
        target_msg.data = (math.degrees(self._target) % 360.0
                           if self._active and self._target is not None else math.nan)
        self._target_publisher.publish(target_msg)

    # --------------------------------------------------------------- helpers

    def _reset_state(self):
        self._target = None
        self._pilot_last_active = None
        self._integral = 0.0

    def _fresh_rate_command(self, now) -> float:
        if self._rate_command_stamp is None:
            return 0.0
        timeout_s = float(self.get_parameter('rate_command_timeout_s').value)
        if self._age_s(self._rate_command_stamp, now) > timeout_s:
            return 0.0
        if abs(math.degrees(self._rate_command)) < _RATE_COMMAND_DEADBAND_DPS:
            return 0.0
        return self._rate_command

    def _pilot_torque_active(self, now) -> bool:
        if self._pilot_stamp is None:
            return False
        timeout_s = float(self.get_parameter('pilot_input_timeout_s').value)
        if self._age_s(self._pilot_stamp, now) > timeout_s:
            return False
        deadband = float(self.get_parameter('pilot_yaw_deadband_Nm').value)
        return abs(self._pilot_torque_Nm) > deadband

    def _orientation_fresh(self, now) -> bool:
        if self._heading_stamp is None:
            return False
        timeout_s = float(self.get_parameter('orientation_timeout_s').value)
        return self._age_s(self._heading_stamp, now) <= timeout_s

    @staticmethod
    def _age_s(stamp, now) -> float:
        return (now - stamp).nanoseconds / 1e9

    @staticmethod
    def _is_valid_smoothing_factor(value: float) -> bool:
        return math.isfinite(value) and 0.0 <= value <= 1.0


def main(args=None):
    rclpy.init(args=args)
    node = HeadingHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
