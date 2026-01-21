Drive Velocity Control – Requirements Specification
1. System Overview

본 프로젝트는 작업 차량(또는 이동 로봇)의 주행 속도 제어를 목표로 하며,
운전자 입력에 따라 안정적이고 예측 가능한 속도 응답을 제공하는 것을 목적으로 한다.

본 제어기는 단순한 속도 추종뿐 아니라,
통신 장애·비상정지·Fault 발생 시 출력을 안전 상태로 전환하도록 설계되었다.

2. Control Objective

목표 속도에 대해 빠르고 안정적으로 수렴

과도한 오버슈트 및 진동 방지

제어 출력 포화 상황에서도 안정성 유지

Fault 발생 시 즉각적인 안전 정지 보장

3. Functional Requirements (FR)
ID	Requirement
FR-01	운전 입력(drive_enable)이 유지되는 동안 목표 속도를 추종해야 한다
FR-02	운전 입력이 해제되면 제어 출력은 즉시 0으로 감소해야 한다
FR-03	통신 장애, 배터리 이상, Fault 발생 시 출력은 강제로 0이 되어야 한다
FR-04	비상 정지(E-STOP) 입력 시 모든 구동 출력은 즉시 차단되어야 한다
FR-05	Fault는 래치되며, 원인 제거 + 사용자 ACK 후에만 해제된다
4. Performance Requirements (PR)

속도 목표: 1.0 (normalized unit)
제어 주기: 10 ms

ID	Metric	Requirement
PR-01	Rise Time (0 → 90%)	≤ 1.0 s  // 작업 효율성: 조작 후 1초 내 반응 필요
PR-02	Overshoot	≤ 5 %            // 적재물 안정성 및 탑승자 안전
PR-03	Settling Time (±5%)	≤ 2.0 s  
PR-04	Steady-State Error	≤ ±0.02
PR-05	Output Saturation Duration	≤ 300 ms
PR-06	Fault 발생 시 출력 차단 시간	≤ 1 control cycle
5. Controller Design

제어 구조: Velocity PID Controller

출력 범위: motor_cmd ∈ [-1.0, 1.0]

출력 포화에 대비한 Anti-Windup 로직 적용

Derivative 항은 노이즈 영향을 고려하여 제한적으로 사용

6. Tuning Strategy

P 제어 단독으로 기본 응답 확보

I 항을 최소한으로 추가하여 정상 상태 오차 제거

Overshoot 또는 출력 토글 발생 시 Gain 증가 중단

성능 요구사항(PR)을 만족하는 최소 Gain 조합을 최종 채택

7. Validation Method

Step 입력(0 → 1.0)에 대한 시뮬레이션 로그 분석

Rise time, Overshoot, Settling time을 로그 기반으로 산출

요구사항(PR) 대비 PASS / FAIL 판정

8. Final Result (Example)
PID Gains: Kp = 2.0, Ki = 1.0, Kd = 0.5
Result: All performance requirements satisfied
Conclusion: Gains accepted as final configuration

9. Conclusion

본 프로젝트는 단순 PID 구현이 아닌,
요구사항 정의 → 성능 기준 설정 → 검증 기반 튜닝의 흐름으로 설계되었다.

이는 산업용 제어 시스템에서 요구되는
안정성, 예측 가능성, 안전성 중심의 설계 사고를 반영한 결과이다.