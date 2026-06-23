# [202021314-배준원] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

## 1. 설계 개요

이 과제의 구현 목표는 BMW_5 기반 14자유도 차량 모델 위에서 도는 통합 섀시 제어기다. 운전자만 운전하는 상태(제어기 OFF)를 기준으로 두고, 거기에 횡·종·수직 제어를 얹어서 핸들링 안정성과 제동, 승차감이 실제로 얼마나 좋아지는지를 KPI로 보이는 게 목표였다. 구체적으로는 능동 전륜 조향(AFS), 안정성 제어(ESC), ABS, 연속 가변 감쇠(CDC), 그리고 이 네 가지의 명령을 실제 액추에이터로 나눠주는 조율기(Coordinator)를 설계했다.

가장 고민이 많았던 건 횡방향 제어기였다. 배점도 여기 몰려 있고, 시나리오 대부분이 횡방향 거동을 본다. 처음엔 강의에서 익숙한 PID로 갈까 했는데, A3(스텝 조향) 채점 기준을 보니 오버슈트·상승시간·정착시간 세 개를 동시에 맞춰야 했다. PID로 이걸 다 잡으려면 상승시간을 줄일 때 오버슈트가 따라 커지는 문제가 생긴다. 그래서 **LQR**로 정했다. LQR은 상태 가중 Q와 입력 가중 R만 잘 잡으면 이 세 지표를 한꺼번에 조율할 수 있고, 무엇보다 차량 상태공간이 속도에 따라 변하기 때문에 속도별로 게인을 다시 계산하면 그게 곧 이득 스케줄링이 된다 [3]. 상태피드백이라 슬립각 제한이나 적분 보정 같은 걸 모듈로 덧붙이기도 편했다.

네 제어기를 한 줄씩 정리하면 이렇다.

- **ctrl_lateral** — 속도 적응 LQR로 yaw rate를 추종(AFS)하고, 적분으로 정상상태 오차를 없애며, 슬립각이 임계를 넘으면 ESC yaw moment를 건다.
- **ctrl_longitudinal** — PI로 속도를 추종하고, 휠 슬립이 커지면 제동력을 줄이는 ABS를 넣고, 저크를 제한한다.
- **ctrl_vertical** — skyhook과 groundhook을 섞은 반능동 감쇠.
- **ctrl_coordinator** — yaw moment를 4륜 제동으로 나눌 때 가중 최소자승(WLS) 방식을 쓰고, 마찰원 제한을 건다.

---

## 2. 수학적 모델링

### 2.1 설계에 쓴 모델

제어기 설계 자체는 2자유도 자전거 모델 위에서 했고, 검증은 과제가 제공한 14DOF 위에서 했다. 둘을 분리한 이유는, LQR을 설계하려면 선형 상태공간이 필요한데 14DOF는 너무 복잡해서 그대로 쓰기 어렵기 때문이다. 자전거 모델은 횡방향 거동의 핵심인 yaw와 sideslip의 결합을 잘 담으면서도 깔끔한 선형 형태를 준다. 물론 14DOF에는 타이어 비선형이나 하중 이동, 롤·피치가 다 들어 있어서 자전거 모델의 가정을 벗어나는데, 그래서 게인에 마진을 두고 14DOF에서 실제로 돌려보며 조정하는 식으로 그 간극을 메웠다.

### 2.2 상태공간

상태를 $x = [v_y,\ r]^T$ (횡속도, yaw rate), 입력을 $u = \delta$ (전륜 조향각)로 두면 다음과 같다.

$$\dot{x} = A x + B u, \qquad y = C x + D u$$

$$
A = \begin{bmatrix}
-\dfrac{C_f + C_r}{m v_x} & -v_x - \dfrac{C_f l_f - C_r l_r}{m v_x} \\[2mm]
-\dfrac{C_f l_f - C_r l_r}{I_z v_x} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v_x}
\end{bmatrix},
\qquad
B = \begin{bmatrix} \dfrac{C_f}{m} \\[2mm] \dfrac{C_f l_f}{I_z} \end{bmatrix}
$$

파라미터는 $m=1500\,\mathrm{kg}$, $I_z=2500\,\mathrm{kg\,m^2}$, $l_f=1.2\,\mathrm{m}$, $l_r=1.4\,\mathrm{m}$, $C_f=80000\,\mathrm{N/rad}$, $C_r=85000\,\mathrm{N/rad}$를 썼다. 여기서 눈여겨볼 점은 $A$와 $B$가 종속도 $v_x$를 품고 있다는 것이다. 즉 속도가 바뀌면 시스템 자체가 바뀌므로, 게인도 속도마다 달라져야 한다는 결론이 자연스럽게 나온다. 이게 뒤에 나올 이득 스케줄링의 근거다.

목표 yaw rate는 운전자 조향각 $\delta_{drv}$로부터 정상상태 자전거 모델로 계산했다.

$$r_{ref} = \frac{v_x\,\delta_{drv}}{L + K_{us} v_x^2}, \qquad K_{us} = \frac{m l_r}{2 C_f L} - \frac{m l_f}{2 C_r L}$$

여기서 $L = l_f + l_r$이고, 언더스티어 계수 $K_{us}$가 속도가 올라갈수록 정상상태 이득이 줄어드는 효과를 담는다 [3]. 이 식은 제공된 `calc_ref_yaw_rate.m`과 같은 형태라, 제어기가 추종하려는 목표와 채점이 보는 기준이 일관되도록 맞췄다.

### 2.3 가정과 한계

설계에서 깔고 간 가정은 세 가지다. 첫째, 종속도를 구간상수로 봤다 — 다만 매 스텝 현재 $v_x$로 상태공간을 다시 만들어 이 가정을 상당히 완화했다. 둘째, 타이어를 선형(소슬립)으로 봤는데, 이건 14DOF에서 슬립이 커지면 어긋나는 부분이다. 셋째, 횡·종·수직이 약하게 결합한다고 보고 따로 설계한 뒤 Coordinator에서 액추에이터 레벨로 다시 합쳤다.

---

## 3. 제어기 설계

### 3.1 ctrl_lateral — AFS + ESC

목표는 yaw rate를 빠르고 안정적으로 추종(정착 0.8초, 오버슈트 10% 이내)하면서, 차체 슬립각이 3°를 넘으면 ESC를 개입시키는 것이었다.

**AFS는 LQR로 짰다.** 비용함수

$$J = \int_0^\infty \left( x^T Q x + u^T R u \right) dt, \qquad Q = \mathrm{diag}(1,\ 500),\ R = 1$$

를 최소화하는 상태피드백 $u=-Kx$를 구하는데, 이는 연속시간 리카티 방정식

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

을 풀어 $K = R^{-1}B^T P$로 얻는다. yaw rate 쪽 가중을 500으로 크게 준 건 추종이 가장 중요했기 때문이고, 이 값은 A3 응답을 보며 정했다.

여기서 한 가지 문제가 있었다. 캠퍼스 MATLAB에 Control System Toolbox의 `lqr`이 항상 있으리란 보장이 없어서, 그게 없어도 돌아가게 만들어야 했다. 그래서 해밀토니안 행렬 $H = \begin{bmatrix} A & -BR^{-1}B^T \\ -Q & -A^T \end{bmatrix}$의 안정 고유공간(실수부가 음인 고유값에 대응하는 고유벡터)으로부터 $P$를 직접 푸는 솔버를 넣었다. 표준 해와 비교했을 때 기계정밀도 수준($10^{-14}$)으로 일치하는 걸 확인했으니, 정확도는 문제없다.

정상상태 오차를 없애려고 yaw rate 오차 적분을 더했고, 적분값이 발산하지 않게 anti-windup으로 잘라줬다.

$$\delta_{AFS} = K\begin{bmatrix} -\hat{v}_y \\ r_{ref}-r \end{bmatrix} + K_i \int (r_{ref}-r)\,dt, \qquad \hat{v}_y = v_x \tan\beta$$

$v_y$는 직접 재기 어려워서 $v_x \tan\beta$로 근사했다.

**이득 스케줄링**은 앞서 말한 대로, 매 스텝 그때의 $v_x$로 $(A,B)$를 다시 만들고 LQR을 재계산하는 식이다. 다만 매번 리카티를 푸는 건 비싸서, $v_x$가 거의 안 변했으면 직전 게인을 그대로 쓰게 했다. 닫힌루프 극을 5~40 m/s 범위에서 확인해보니 전부 좌반면에 있었고(저속에서 −24.5, 고속에서 −3.45 rad/s), 둘 다 정착시간 0.8초 안에 들어오는 값이라 안심했다.

**ESC는 β-limiter 형태**로 짰다. 슬립각이 임계 $\beta_{th}=3°$를 넘으면, 넘은 만큼에 비례하고 속도로 스케일한 복원 모멘트를 운전자 의도와 반대로 건다.

$$M_z = -K_\beta\,\mathrm{sign}(\beta)\,(|\beta|-\beta_{th})\,f(v_x), \qquad f(v_x)=\min(v_x/20,\ 2)$$

**AFS 권한 제한**은 사실 처음엔 없었는데, 돌려보니 필요해서 넣은 부분이다. 폐루프(경로추종) 운전자가 도는 시나리오에서 AFS가 yaw rate를 쫓느라 운전자 조향 위에 보조 조향을 크게 얹으면, 운전자가 따라가려는 경로와 싸우면서 경로를 벗어났다. 그래서 보조 조향에 상한을 뒀다. 단 ESC yaw moment는 제한하지 않았는데, 이건 A7 같은 가혹 기동에서 스핀아웃을 막는 핵심이라 묶으면 안 된다. 이 상한값은 안정성과 경로추종이 맞바뀌는 지점이라 여러 값으로 시험했고, 30°일 때 sideSlip·LTR이 가장 좋게 나왔다(이 과정은 5장에 자세히 적었다).

최종 게인은 `sim_params.m`에 이렇게 넣었다.
```matlab
CTRL.LAT.Q             = [1 0; 0 500];
CTRL.LAT.R             = 1.0;
CTRL.LAT.Ki_lqr        = 2.0;
CTRL.LAT.afsAuthority  = deg2rad(30);
CTRL.LAT.betaThreshold = deg2rad(3);
CTRL.LAT.betaGain      = 6e4;
```

### 3.2 ctrl_longitudinal — 속도 + ABS

속도 추종은 PI로 가속도 명령을 만들고 $F_x = m\,a_{cmd}$로 힘으로 바꿨다. ABS는 휠 슬립비 $|\kappa|$가 목표 0.12를 넘고 감속 중일 때 제동력을 줄이는 방식인데, 켜고 끄는 bang-bang보다 연속적으로 줄이는 쪽이 슬립 RMS에 유리할 것 같아 그렇게 했다. 저크는 $|\dot{F}_x| \le m\cdot\mathrm{JERK_{max}}$로 제한하고 적분은 역시 anti-windup으로 잘랐다. 슬립비는

$$\kappa = \frac{\omega r_w - v_x}{\max(v_x,\,0.1)}$$

로 정의된다. 휠 슬립 자체는 이 함수에 직접 안 들어와서, runner가 직전 스텝 값을 `ctrlState.wheelSlip`에 넣어주는 걸 받아 쓰도록 했다.

### 3.3 ctrl_vertical — CDC

skyhook과 groundhook을 섞은 반능동 감쇠를 휠마다 따로 적용했다. skyhook은 차체(스프렁) 속도를 줄여 승차감을 챙기고, groundhook은 바퀴(언스프렁) 속도를 줄여 접지를 챙긴다. 둘을 블렌딩한 형태는

$$c_i = \alpha\,c_{sky,i} + (1-\alpha)\,c_{ground,i}, \qquad c_{min} \le c_i \le c_{max}$$

이고, 차체 속도와 상대 속도의 곱이 양이면 크게, 아니면 작게 감쇠하는 on-off skyhook 논리를 바탕에 깔았다. $\alpha=0.8$로 skyhook 비중을 높여 승차감 쪽에 무게를 뒀다.

### 3.4 ctrl_coordinator — WLS Allocation + 마찰원 제한

종방향 제동은 단순하다. $F_x<0$이면 총 제동 토크를 전후 60:40으로 네 바퀴에 나눈다.

**ESC yaw moment를 나누는 데는 WLS를 썼다.** 요청된 yaw moment $M_z$를 4륜 추가 제동력 $u=[dF_{FL},dF_{FR},dF_{RL},dF_{RR}]^T$로 배분하는데, 제동력이 yaw moment를 만드는 관계는

$$M_z = B_{alloc}\,u, \qquad B_{alloc} = \left[-\tfrac{t_f}{2},\ +\tfrac{t_f}{2},\ -\tfrac{t_r}{2},\ +\tfrac{t_r}{2}\right]$$

이다. 이걸 그냥 좌우로 절반씩 나눠도 되지만, 액추에이터를 얼마나 쓰는지를 같이 최소화하고 싶어서 가중 최소자승 문제로 풀었다.

$$\min_u\ \|W^{1/2} u\|^2 \ \ \text{s.t.}\ \ B_{alloc} u = M_z \quad\Rightarrow\quad u^\star = W^{-1} B_{alloc}^T \left(B_{alloc} W^{-1} B_{alloc}^T\right)^{-1} M_z$$

가중 $W=\mathrm{diag}(w_F,w_F,w_R,w_R)$에서 후축 가중을 전축보다 크게 줘서($w_R>w_F$), 조향축인 전축을 차동 제동에 더 쓰도록 했다. 한 가지 주의할 점이 있었는데, WLS는 노력을 최소화하는 해라서 좀 보수적으로 나온다. 그대로 쓰면 ESC가 약해져서 A7 성능이 떨어질 수 있었다. 그래서 강도 게인 $\mathrm{escGain}$으로 스케일했고, 이렇게 하니 단순 차동 분배와 같은 yaw 권한을 유지하면서(A7 sideSlip이 그대로였다) WLS 구조의 이점만 챙길 수 있었다 [6].

**마찰원 제한**은 각 휠의 제동 토크가 그 휠이 낼 수 있는 마찰력 한계 $T_{max}=\mu F_z r_w$를 넘지 않게 막는 것이다(전륜 $F_{z,f}=mg\,l_r/2L$, 후륜 $F_{z,r}=mg\,l_f/2L$로 정적 하중을 근사). 마지막엔 $[0,\ \mathrm{MAX\_BRAKE\_TRQ}]$로 잘라준다.

---

## 4. 시뮬레이션 결과

### 4.1 P1 시나리오 — 베이스라인 vs 설계 제어기

14DOF, dry 노면에서 제어기 OFF와 ON을 비교한 결과다.

| 시나리오 | KPI | OFF | ON | 개선 |
|---|---|---:|---:|---:|
| A1 ISO 3888-1 DLC | sideSlipMax [°] | 3.015 | 1.755 | −41.8 % |
| A1 | LTR_max [-] | 0.864 | 0.473 | −45.3 % |
| A3 ISO 7401 Step | yawRateOvershoot [%] | 2.70 | 2.65 | 통과 |
| A3 | yawRateRiseTime [s] | 0.247 | 0.093 | −62.3 % |
| A3 | yawRateSettling [s] | 1.462 | 0.585 | −60.0 % |
| A4 ISO 4138 SS Circular | understeerGradient | 0.0007 | 0.0007 | 기준 내 |
| A7 ISO 7975 Brake-in-Turn | sideSlipMax [°] | 30.48 | 1.728 | −94.3 % |
| A7 | LTR_max [-] | 0.681 | 0.318 | −53.3 % |
| D1 DLC+Brake | sideSlipMax [°] | 4.906 | 1.755 | −64.2 % |
| D1 | LTR_max [-] | 0.864 | 0.469 | −45.7 % |

자동 채점으로는 정량 **49.00 / 70.00**, 감점은 0이었다. A3와 A7에서 만점(각 12/12, 15/15)을 받았고, A1·D1에서는 안정성 지표인 sideSlip과 LTR이 만점이었다.

### 4.2 A3 스텝 응답 — LQR이 가장 잘 보이는 곳

A3는 80 km/h에서 조향을 2° 계단 입력으로 주고 yaw rate가 얼마나 빠르고 깔끔하게 반응하는지를 본다. LQR을 쓴 이유가 가장 직접적으로 드러나는 시나리오라 따로 본다.

상승시간이 0.247초에서 0.093초로 62% 줄었고, 정착시간은 1.462초에서 0.585초로 60% 줄어 기준(0.8초)을 통과했다. 오버슈트는 2.65%로 기준(10%) 안에 넉넉히 들어왔다. Q에서 yaw rate 가중을 크게 준 게 빠른 추종을 만들었고, 적분이 정상상태 오차를 지웠으며, 극이 좌반면 깊숙이 박힌 덕에 진동 없이 정착했다. 세 효과가 맞물린 결과다.

### 4.3 가혹 기동 — A7과 A5

**A7(선회 중 제동)** 이 개인적으로 가장 인상적이었다. 제어기 없이 돌리면 sideSlip이 30.5°까지 올라가 차가 사실상 돌아버린다(스핀아웃). 여기에 제어기를 켜니 ESC가 슬립각 초과를 잡아내고 WLS 차동 제동이 복원 모멘트를 걸어서 sideSlip을 1.73°로 눌렀다. 94% 개선이다. LTR도 0.68에서 0.32로 떨어져 전복 위험이 크게 줄었다.

**A5(FMVSS 126 Sine-with-Dwell)** 는 가산점을 노리고 추가로 돌려본 시나리오다. 베이스라인은 sideSlip이 무려 70.9°로 완전히 스핀아웃하는데, 제어기를 켜니 1.53°까지 잡혔다(98% 개선). FMVSS 기준 중 후기 응답을 보는 R1.75는 0.855에서 0.027로 떨어져 기준(0.20)을 가뿐히 통과했다. 다만 R1.0(조향 정점 1초 후)은 0.987로 기준 0.35를 못 맞춰서, 최종 FMVSS pass 판정까지는 가지 못했다. 원인을 따져보면, AFS가 yaw rate를 운전자 조향 패턴에 충실히 맞추다 보니, 조향이 dwell 구간에서 급히 0으로 돌아올 때 yaw가 그만큼 빨리 안 죽는 것 같다. 그래도 완전한 스핀을 막은 것만으로 ESC가 제 역할을 했다는 건 분명하다.

### 4.4 이득 스케줄링과 WLS의 효과

이득 스케줄링 덕에 yaw rate 게인이 속도(5~40 m/s)에 따라 단조롭게 변하면서, 넓은 속도 영역에서 추종이 일관되게 유지됐다. WLS는 앞서 말한 대로 단순 차동과 같은 yaw 권한을 내면서도 액추에이터 노력을 최소화하는 최적성을 더했고, 마찰원 제한과 함께 작동해 휠 잠김을 억제했다.

## 5. 분석 + 한계

### 5.1 가장 잘 된 시나리오

단연 **A7**이다. 30.5°짜리 스핀아웃을 1.73°로 잡은 94% 개선이 가장 컸고, 의도한 ESC와 WLS 조합이 그대로 먹혀서 만점을 받았다. A5에서 70.9°를 1.5°로 잡은 것도 같은 메커니즘이 더 극단적인 상황에서 작동한 사례다. 가혹할수록 잘 잡혔다는 게 흥미로웠다.

### 5.2 안 된 것과 그 이유

**B1(직선 제동)** 과 **경로 이탈(lateralDev)** 지표에서는 점수를 얻지 못하였다. 

원인을 그대로 적어보자면

B1 정지거리는 72.3 m가 나왔는데, 이건 제어기로 컨트롤하기 어려웠다. B1은 운전자가 그냥 브레이크만 밟는 open-loop 시나리오라, 제동력을 시나리오가 강제로 정한다. 제어기가 총 제동을 키우거나 줄일 통로가 없고, 정지거리는 결국 타이어 마찰과 차량이 결정한다. 디버깅하면서 보니 뒷바퀴가 슬립 −1.0, 즉 완전히 잠기는 것까지 확인했는데, runner가 시나리오 제동에 제어기 출력을 더하는 구조라(`brk_scenario + brakeESC`) 뒷바퀴 제동을 직접 깎아낼 방법이 마땅치 않았다.

경로 이탈은 1.83 m에서 2.20 m로 오히려 늘었다. AFS가 yaw rate를 쫓느라 운전자 조향 위에 보조 조향을 얹으면서 경로추종과 부딪힌 결과다. 그런데 따져보니 lateralDev 기준이 0.7 m인데 베이스라인부터 이미 1.83 m로 한참 초과 상태였다. 즉 이 점수는 제어기를 어떻게 해도 받기 어려운 구조라, 안정성(sideSlip·LTR)을 챙기는 쪽으로 방향을 잡았다. 실제로 AFS 권한을 5°까지 낮춰 경로 이탈을 줄여보려 했더니, sideSlip과 LTR이 같이 나빠졌다(A1의 LTR이 0.47에서 0.72로 올라 기준을 넘었다). 안정성을 만드는 게 결국 AFS였던 셈이라, 안정성 지표 만점을 지키는 30° 설정을 택했다. 점수를 두고 보면 이게 맞는 선택이었다.

### 5.3 시간이 더 있었다면

세 가지를 더 해보고 싶었다. 첫째, A5의 R1.0을 잡기 위해 dwell 구간에서 yaw rate 자체를 0으로 끌어당기는 항을 넣어보는 것 — 다만 이게 A3 추종을 해치지 않는지 확인이 필요하다. 둘째, B1 뒷바퀴 잠김을 풀려면 Coordinator가 시나리오 제동을 상쇄하는 음의 토크를 휠별로 거는 구조를 검토해야 한다. 셋째, 안정성과 경로추종이 맞바뀌는 근본 문제를 풀려면, 정상상태인지 아닌지를 감지해서 AFS를 자동으로 줄였다 키웠다 하는 적응 로직이 답일 것 같다.

---

## 6. 참고문헌

[1] ISO 3888-1:2018, *Passenger cars — Test track for a severe lane-change manoeuvre — Part 1: Double lane-change.*
[2] ISO 7401:2011, *Road vehicles — Lateral transient response test methods.*
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012. (§2.5 yaw rate response, §3 LQR, §8 ESC)
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008.
[5] NHTSA FMVSS 126 / ISO 19365:2016, *Sine-with-dwell stability test.*
[6] T. A. Johansen and T. I. Fossen, "Control allocation — A survey," *Automatica*, vol. 49, no. 5, 2013.
[7] D. Karnopp et al., "Vibration control using semi-active force generators," *J. Eng. Industry*, 1974.

---

## 부록 A — 사용한 AI 도구

이 과제에서 Anthropic Claude를 설계와 게인 튜닝 보조에 사용했다. 주로 LQR 가중 행렬 초기값을 잡고, Toolbox 없이 도는 CARE 솔버를 구현하고, WLS allocation을 정식화하고, 시뮬레이션 결과를 해석해 게인을 어느 방향으로 움직일지 정하는 데 도움을 받았다. 다만 모든 코드는 14DOF에서 직접 돌려 검증했고, `sim_params.m`의 게인도 실제 결과를 보며 반복해서 조정해 최종값을 정했다.

## 부록 B — sim_params.m 변경사항

수정은 허용 범위인 `CTRL.*` 항목만 수정하였다. 추가한 게인은 다음과 같다.

```matlab
% 횡방향 게인 추가
CTRL.LAT.Q             = [1 0; 0 500];   % LQR 상태 가중 (yaw rate 강조)
CTRL.LAT.R             = 1.0;            % LQR 입력 가중
CTRL.LAT.Ki_lqr        = 2.0;           % yaw rate 적분 게인
CTRL.LAT.afsAuthority  = deg2rad(30);   % AFS 권한 한계
CTRL.LAT.betaThreshold = deg2rad(3);    % ESC 개입 임계
CTRL.LAT.betaGain      = 6e4;           % ESC yaw moment 게인

% Coordinator 게인 추가 (WLS allocation)
CTRL.COORD.wlsFrontWeight = 1.0;        % WLS 전축 effort 가중
CTRL.COORD.wlsRearWeight  = 1.5;        % WLS 후축 effort 가중
CTRL.COORD.escGain        = 3.0;        % ESC 강도 스케일
```
