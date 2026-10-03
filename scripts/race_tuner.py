#!/usr/bin/env python3
"""Safely change race-controller tuning while race.launch.py is running."""
import argparse

import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


def main():
    parser = argparse.ArgumentParser(description="运行中调整灵动越障参数")
    parser.add_argument("--speed-scale", type=float, help="速度倍率 0.20~1.00")
    parser.add_argument("--normal-speed", type=float, help="最高巡线速度，最大 0.50 m/s")
    parser.add_argument("--min-speed", type=float, help="急弯最低速度")
    parser.add_argument("--max-angular", type=float, help="最大转向速度")
    parser.add_argument("--curve-slowdown", type=float, help="弯道减速系数")
    parser.add_argument("--s-curve-speed", type=float, help="连续 S 弯速度，0.05~0.50 m/s")
    parser.add_argument("--s-curve-min-speed", type=float, help="连续 S 弯最低速度")
    parser.add_argument("--s-curve-max-angular", type=float, help="连续 S 弯最大角速度")
    parser.add_argument("--roundabout-filter", type=float, help="环岛角速度平滑系数，0.05~0.95")
    parser.add_argument("--roundabout-marker-speed", type=float, help="首次环岛标记直行速度；等待时间自动换算以保持通过距离")
    parser.add_argument("--roundabout-marker-nominal-seconds", type=float, help="首次标记通过距离：默认 1.0 秒 × 0.08m/s = 0.08m")
    parser.add_argument("--roundabout-min-ccw", type=float, help="环岛最小逆时针角速度，阻止虚线把车拉回直线")
    parser.add_argument("--solid-reacquire-speed", type=float, help="实线丢失找回速度，0.03~0.25 m/s")
    parser.add_argument("--solid-reacquire-min-speed", type=float, help="实线找回最低速度")
    parser.add_argument("--solid-reacquire-max-angular", type=float, help="实线找回最大角速度")
    parser.add_argument("--roundabout-exit-speed", type=float, help="环岛 360°后直行找实线速度")
    parser.add_argument("--height", type=float, help="巡线腿高，范围 0.14~0.36 m")
    parser.add_argument("--pitch", type=float, help="巡线俯仰角")
    args = parser.parse_args()
    mapping = {
        "speed_scale": args.speed_scale, "normal_speed": args.normal_speed,
        "min_speed": args.min_speed, "max_angular_speed": args.max_angular,
        "curve_slowdown": args.curve_slowdown, "track_height_m": args.height,
        "track_pitching_deg": args.pitch,
        "s_curve_speed_m_s": args.s_curve_speed,
        "s_curve_min_speed_m_s": args.s_curve_min_speed,
        "s_curve_max_angular_rad_s": args.s_curve_max_angular,
        "roundabout_angular_filter_alpha": args.roundabout_filter,
        "roundabout_marker_pass_speed_m_s": args.roundabout_marker_speed,
        "roundabout_marker_cooldown_seconds": args.roundabout_marker_nominal_seconds,
        "roundabout_min_ccw_angular_rad_s": args.roundabout_min_ccw,
        "solid_reacquire_speed_m_s": args.solid_reacquire_speed,
        "solid_reacquire_min_speed_m_s": args.solid_reacquire_min_speed,
        "solid_reacquire_max_angular_rad_s": args.solid_reacquire_max_angular,
        "roundabout_exit_search_speed_m_s": args.roundabout_exit_speed,
    }
    parameters = [Parameter(name, value=value) for name, value in mapping.items() if value is not None]
    if not parameters:
        parser.error("至少提供一个调节选项，例如 --speed-scale 0.85")
    rclpy.init()
    node = rclpy.create_node("race_tuner")
    client = AsyncParameterClient(node, "race_controller")
    if not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().error("race_controller 未运行，无法调参")
        node.destroy_node(); rclpy.shutdown(); return
    future = client.set_parameters(parameters)
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    result = future.result()
    if result is None:
        node.get_logger().error("调参请求超时")
    else:
        for parameter, status in zip(parameters, result):
            text = "已生效" if status.successful else "拒绝：" + status.reason
            node.get_logger().info("%s = %s：%s" % (parameter.name, parameter.value, text))
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
