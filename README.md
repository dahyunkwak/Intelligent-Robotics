# Intelligent-Robotics
26-1 
Campus Delivery Robot with Reinforcement Learning
이 프로젝트는 WSL2 환경에서 ROS2 Humble과 Gazebo를 활용하여 캠퍼스 내 보행자와 배터리 상태를 고려해 최적의 의사결정을 내리는 자율주행 배달 로봇 시스템을 설계합니다.
1. 프로젝트 개요
주제: 캠퍼스 환경에서 보행자와 배터리 상태를 실시간으로 판단하여 배달 상황을 결정하는 강화학습 기반 자율주행 로봇 시스템
목표:
안정성: 보행자와의 충돌 회피 및 안전 거리 유지
효율성: 목표 배달지까지의 신속한 이동
지속성: 실시간 배터리 상태를 고려한 전략적 충전 및 운용
2. 시스템 아키텍처
강화학습(PPO) 기반의 고수준 의사결정과 기존 Navigation 시스템(Nav2)을 결합한 계층적 구조로 설계되었습니다.
구성 요소
역할
 
Gazebo
캠퍼스 환경 및 보행자(Actor) 시뮬레이션
ROS2 Humble
로봇 상태 및 센서 데이터 처리 미들웨어
Gymnasium
강화학습 환경 인터페이스(Gym Wrapper) 구성
RL Agent (PPO)
배달 지속 여부 또는 충전소 이동 결정 (High-level Decision)
Nav2
에이전트가 결정한 목표지까지의 경로 계획 및 실시간 주행
RViz Marker
로봇 상단 배터리 상태 실시간 시각화

3. 강화학습 설계
3.1 상태 (State)
로봇의 현재 위치 및 목표 위치 (Pose)
Gazebo Actor 기반 보행자의 위치 (Ground-truth)
로봇의 이동 속도 및 현재 배터리 잔량
3.2 행동 (Action)
Action 0: 배달 지속 (Continue Delivery)
Action 1: 충전소 이동 (Go to Charging Station)
3.3 보상 함수 (Reward Function)
배달 측면: 성공 시 (+) 보상, 이동 스텝마다 소량의 (-) 보상 (최단 경로 유도)
안전 측면: 보행자 근접 시 큰 (-) 페널티, 밀집 지역 진입 시 위험 가중치 부여
지속성 측면: 배터리 방전 시 큰 (-) 페널티, 충전소 도달 및 충전 시 (+) 보상
4. 구현 과정
ROS2 및 Gazebo 기반 캠퍼스 시뮬레이션 환경 구축
Gazebo Actor를 활용한 보행자 생성 및 위치 데이터 추출
Gymnasium 기반 강화학습 환경 구축 및 MDP 정의
배터리 소모/회복 로직 구현 및 RViz 마커 연동
Stable Baselines3의 PPO 알고리즘을 활용한 정책 학습
학습된 에이전트와 Nav2 시스템 통합 테스트
5. 참고 자료
ROS2 Navigation (Nav2): https://docs.nav2.org/
Proximal Policy Optimization (PPO): https://arxiv.org/abs/1707.06347
Gymnasium: https://gymnasium.farama.org/
