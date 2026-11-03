#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
강의명: 오토모티브SW프로그래밍
주차: 9주차
파일명: custom_marker_publisher.py
작성: 컴퓨터공학 / 21114020 / 조민서
설명: 가상 주행 경로(LINE_STRIP), 3D 장애물(CUBE), 위험도 안내 텍스트(TEXT_VIEW_FACING),
      그리고 과거 5초 주행 궤적(POINTS, 알파 그라데이션)을 생성하여
      MarkerArray 토픽으로 실시간 발행하는 RViz2 심화 시각화 노드.

[기본 과제 대비 추가 구현 — 도전 미션]
      차량의 과거 5초간 이동 궤적을 0.5초 간격으로 샘플링하고,
      오래된 점일수록 알파(투명도)가 낮아지는 그라데이션 POINTS 마커로 시각화.
      가상 오도메트리를 내부에서 적분해 월드 좌표를 유지한 뒤,
      매 프레임 현재 차량 기준(base_link)으로 역변환하여 발행한다.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from builtin_interfaces.msg import Duration


class CustomMarkerPublisherNode(Node):
    """
    자율주행 차량의 경로 계획 및 센서 인식 결과를
    RViz2 3D 마커(MarkerArray)로 실시간 변환 및 퍼블리시하는 노드
    """

    def __init__(self):
        super().__init__('custom_marker_publisher')

        # =====================================================================
        # 1. 파라미터 선언 (외부 Launch 및 YAML에서 동적 변경 가능)
        # =====================================================================
        self.declare_parameter('frame_id', 'base_link')    # 기준 좌표계 (차량 기준 프레임)
        self.declare_parameter('publish_rate', 10.0)       # 토픽 발행 주기 (Hz)
        self.declare_parameter('obstacle_distance', 4.0)   # 전방 장애물 초기 거리 (m)

        # --- 도전 미션용 파라미터 ---
        self.declare_parameter('trail_duration', 5.0)      # 궤적 보관 시간 (초)
        self.declare_parameter('trail_interval', 0.5)      # 궤적 샘플링 간격 (초)
        self.declare_parameter('vehicle_speed', 1.2)       # 가상 차량 전진 속도 (m/s)
        self.declare_parameter('vehicle_yaw_rate', 0.22)   # 가상 차량 요 각속도 (rad/s)

        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value

        self.trail_duration = self.get_parameter('trail_duration').value
        self.trail_interval = self.get_parameter('trail_interval').value
        self.vehicle_speed = self.get_parameter('vehicle_speed').value
        self.vehicle_yaw_rate = self.get_parameter('vehicle_yaw_rate').value

        # =====================================================================
        # 2. 토픽 퍼블리셔 생성
        #    - 복수의 마커를 묶어서 한 번에 보내는 MarkerArray 사용 (네트워크 효율 최적화)
        # =====================================================================
        self.marker_array_pub = self.create_publisher(
            MarkerArray,
            '/visual_markers',
            10
        )

        # =====================================================================
        # 3. 주기적 발행 타이머 가동 (10Hz)
        # =====================================================================
        timer_period = 1.0 / self.publish_rate
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 가상 주행 경로 및 장애물 동적 움직임을 시뮬레이션하기 위한 시간 변수
        self.time_step = 0.0

        # ---------------------------------------------------------------------
        # [도전 미션] 가상 오도메트리 상태와 궤적 버퍼
        #   - 마커는 base_link(차량 기준) 좌표로 발행되므로,
        #     과거 위치를 그대로 쓰면 차량을 따라 움직여 궤적이 되지 않는다.
        #   - 그래서 월드 좌표(odom_x, odom_y, odom_yaw)를 내부에서 적분해 두고,
        #     발행 시점마다 "현재 차량 기준"으로 역변환해 뒤로 흘러가게 만든다.
        # ---------------------------------------------------------------------
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.elapsed = 0.0                 # 노드 기동 후 누적 시간 (초)
        self.last_sample_t = -1.0          # 마지막으로 궤적을 샘플링한 시각

        # 5초 ÷ 0.5초 = 10개 + 현재 위치 1개 → 여유분 포함 maxlen
        max_samples = int(self.trail_duration / self.trail_interval) + 2
        self.trail = deque(maxlen=max_samples)   # (시각, 월드x, 월드y)

        self.get_logger().info(
            f'=== 커스텀 마커 퍼블리셔 노드 시작 ===\n'
            f'  - 기준 좌표계(frame_id): {self.frame_id}\n'
            f'  - 발행 주기: {self.publish_rate:.1f} Hz\n'
            f'  - 장애물 기준 거리: {self.obstacle_distance:.2f} m\n'
            f'  - 궤적 보관: {self.trail_duration:.1f}초 / '
            f'{self.trail_interval:.1f}초 간격 (최대 {max_samples - 2}개 점)'
        )

    # =========================================================================
    # 가상 오도메트리 적분 (도전 미션)
    # =========================================================================
    def update_virtual_odometry(self):
        """
        차량이 일정 속도로 전진하며 완만히 선회한다고 가정하고
        월드 좌표계 상의 위치와 자세를 매 주기 적분한다.
        실제 차량이라면 /odom 토픽이나 TF2 조회로 대체되는 부분이다.
        """
        self.odom_yaw += self.vehicle_yaw_rate * self.dt
        self.odom_x += self.vehicle_speed * math.cos(self.odom_yaw) * self.dt
        self.odom_y += self.vehicle_speed * math.sin(self.odom_yaw) * self.dt
        self.elapsed += self.dt

        # 0.5초 간격으로만 궤적 점을 적립한다 (요구사항: 0.5초 간격 샘플링)
        if self.elapsed - self.last_sample_t >= self.trail_interval:
            self.trail.append((self.elapsed, self.odom_x, self.odom_y))
            self.last_sample_t = self.elapsed

        # 보관 시간을 넘긴 오래된 점은 버린다
        while self.trail and (self.elapsed - self.trail[0][0]) > self.trail_duration:
            self.trail.popleft()

    def world_to_base_link(self, wx, wy):
        """
        월드 좌표 (wx, wy)를 현재 차량 자세 기준의 base_link 좌표로 변환한다.
        회전행렬의 전치(= 역회전)를 적용하는 2차원 동차 변환이다.
            [ x_local ]   [  cosθ  sinθ ] [ wx - x ]
            [ y_local ] = [ -sinθ  cosθ ] [ wy - y ]
        """
        dx = wx - self.odom_x
        dy = wy - self.odom_y
        c = math.cos(self.odom_yaw)
        s = math.sin(self.odom_yaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    def build_trail_marker(self, stamp, lifetime):
        """
        과거 궤적을 알파 그라데이션 POINTS 마커 하나로 만든다.

        POINTS 타입은 points[] 와 colors[] 를 1:1로 대응시킬 수 있어
        점마다 다른 투명도를 줄 수 있다. 마커를 10개 따로 만들지 않고
        단일 마커로 처리하므로 렌더링 부하도 최소가 된다.
        """
        trail_marker = Marker()
        trail_marker.header.frame_id = self.frame_id
        trail_marker.header.stamp = stamp
        trail_marker.ns = 'trail_history'
        trail_marker.id = 301
        trail_marker.type = Marker.POINTS
        trail_marker.action = Marker.ADD

        # POINTS 는 scale.x = 점의 폭, scale.y = 점의 높이로 쓰인다
        trail_marker.scale.x = 0.20
        trail_marker.scale.y = 0.20

        # 마커 전체 색상은 per-point colors[] 가 덮어쓰므로 기본값만 채워 둔다
        trail_marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=1.0)
        trail_marker.lifetime = lifetime
        trail_marker.pose.orientation.w = 1.0

        for sample_t, wx, wy in self.trail:
            lx, ly = self.world_to_base_link(wx, wy)
            trail_marker.points.append(Point(x=float(lx), y=float(ly), z=0.05))

            # 경과 시간 0초(방금) → 1.0, 5초(가장 오래됨) → 0.0 으로 정규화
            age = self.elapsed - sample_t
            freshness = 1.0 - min(age / self.trail_duration, 1.0)

            # 알파: 0.95(최신) → 0.10(가장 오래됨) 선형 감쇄
            alpha = 0.10 + 0.85 * freshness
            # 색상도 함께 식혀 준다 (밝은 시안 → 짙은 파랑)
            trail_marker.colors.append(ColorRGBA(
                r=0.10,
                g=0.35 + 0.55 * freshness,
                b=1.0,
                a=float(alpha)
            ))

        return trail_marker

    # =========================================================================
    # 주기 콜백
    # =========================================================================
    def timer_callback(self):
        """
        10Hz(0.1초) 주기로 실행되며, 4종류의 마커를 생성하여 MarkerArray로 일괄 발행
        """
        now = self.get_clock().now()
        marker_array = MarkerArray()

        # -------------------------------------------------------------
        # [메모리 누수 및 고스트 오브젝트 방지 핵심]
        # lifetime = 0.2초 설정
        # 10Hz(0.1초 간격)로 새 위치가 덮어써지는 동안은 매끄럽게 유지되지만,
        # 노드가 종료되거나 감지 대상이 사라지면 0.2초 후 RViz2 화면에서 자동 소멸됨
        # -------------------------------------------------------------
        marker_lifetime = Duration(sec=0, nanosec=200000000)  # 0.2초 (200,000,000 ns)

        # 애니메이션 시간 누적
        self.time_step += 0.05

        # 가상 주행 상태 갱신 (도전 미션용 궤적 적립 포함)
        self.update_virtual_odometry()

        # =====================================================================
        # [마커 1] 가상 주행 계획선 (LINE_STRIP)
        # =====================================================================
        path_marker = Marker()
        path_marker.header.frame_id = self.frame_id
        path_marker.header.stamp = now.to_msg()
        path_marker.ns = 'trajectory'
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        # 선의 굵기 (scale.x만 선 두께로 사용됨)
        path_marker.scale.x = 0.08

        # 에메랄드 그린 색상 및 85% 알파 블렌딩 적용
        path_marker.color = ColorRGBA(r=0.1, g=0.9, b=0.4, a=0.85)
        path_marker.lifetime = marker_lifetime
        path_marker.pose.orientation.w = 1.0

        # 전방 10m 곡선 궤적 점군(Point List) 생성
        for i in range(40):
            x = i * 0.25  # 전방 0m부터 10m까지 0.25m 간격
            y = 0.7 * math.sin(0.4 * x + self.time_step)  # 부드러운 S자 주행선
            z = 0.02      # 노면 위에 살짝 띄움 (Z-파이팅 방지)
            path_marker.points.append(Point(x=float(x), y=float(y), z=float(z)))

        marker_array.markers.append(path_marker)

        # =====================================================================
        # [마커 2] 감지된 전방 장애물 3D 바운딩 박스 (CUBE)
        # =====================================================================
        obs_marker = Marker()
        obs_marker.header.frame_id = self.frame_id
        obs_marker.header.stamp = now.to_msg()
        obs_marker.ns = 'obstacles'
        obs_marker.id = 101
        obs_marker.type = Marker.CUBE
        obs_marker.action = Marker.ADD

        # 장애물 3차원 위치 설정 (전방에서 좌우로 천천히 횡단하는 가상 보행자/차량)
        obs_x = float(self.obstacle_distance)
        obs_y = 0.8 * math.sin(self.time_step * 0.8)
        obs_z = 0.5  # 바닥에서 0.5m 높이 (박스 중심 좌표)

        obs_marker.pose.position.x = obs_x
        obs_marker.pose.position.y = obs_y
        obs_marker.pose.position.z = obs_z
        obs_marker.pose.orientation.w = 1.0  # 회전 없음 (기본 자세)

        # 박스 크기 (가로 0.8m, 세로 0.8m, 높이 1.0m)
        obs_marker.scale.x = 0.8
        obs_marker.scale.y = 0.8
        obs_marker.scale.z = 1.0

        # 반투명 오렌지-레드 경고 색상 (내부 관찰 가능하도록 알파 60% 적용)
        obs_marker.color = ColorRGBA(r=1.0, g=0.35, b=0.1, a=0.6)
        obs_marker.lifetime = marker_lifetime

        marker_array.markers.append(obs_marker)

        # =====================================================================
        # [마커 3] 카메라 시점 직면 경고 텍스트 라벨 (TEXT_VIEW_FACING)
        # =====================================================================
        text_marker = Marker()
        text_marker.header.frame_id = self.frame_id
        text_marker.header.stamp = now.to_msg()
        text_marker.ns = 'warning_labels'
        text_marker.id = 201
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        # 텍스트 위치 (장애물 박스 정중앙 바로 윗부분)
        text_marker.pose.position.x = obs_x
        text_marker.pose.position.y = obs_y
        text_marker.pose.position.z = obs_z + 0.8  # 장애물 상단 0.8m 지점
        text_marker.pose.orientation.w = 1.0

        # 폰트 높이 (scale.z만 사용됨)
        text_marker.scale.z = 0.32

        # 텍스트 내용 계산 (유클리드 거리 산출)
        dist_to_obstacle = math.sqrt(obs_x**2 + obs_y**2)
        # RViz2(OGRE)의 기본 폰트에는 한글 글리프가 없어 한글을 넣으면 빈칸으로 렌더링된다.
        # 강의자료 7단계의 판정 기준도 영문 표기이므로 ASCII로 작성한다.
        text_marker.text = f'WARNING: OBSTACLE\n[{dist_to_obstacle:.2f} m]'

        # 순백색 불투명 텍스트 (가독성 극대화)
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text_marker.lifetime = marker_lifetime

        marker_array.markers.append(text_marker)

        # =====================================================================
        # [마커 4 · 도전 미션] 과거 5초 주행 궤적 (POINTS, 알파 그라데이션)
        # =====================================================================
        marker_array.markers.append(
            self.build_trail_marker(now.to_msg(), marker_lifetime)
        )

        # =====================================================================
        # 4. MarkerArray 토픽 일괄 전송
        #    4종 마커가 단일 패킷·단일 타임스탬프로 원자적으로 전달된다
        # =====================================================================
        self.marker_array_pub.publish(marker_array)


def main(args=None):
    """
    ROS 2 노드 진입점 함수
    """
    rclpy.init(args=args)
    node = CustomMarkerPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 인터럽트로 마커 노드를 정상 종료합니다.')
    finally:
        node.destroy_node()
        # 배포판에 따라 이미 shutdown 된 상태에서 재호출하면 예외가 발생하므로 확인 후 호출
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
