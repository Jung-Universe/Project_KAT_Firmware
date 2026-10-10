#include "Copter.h"

#if MODE_TDCN_ENABLED

#include "mode_tdcn_claw_extern.h"

extern "C" {
volatile uint8_t Arming = 0;
}

// ---------------------------------------------------------------------------
/* 모드 진입시 1회 실행 */
bool ModeTDCN::init(bool ignore_checks)
{
    // 공중 진입 (예: 6 -> Loiter -> TDCN) 은 state 5 에서 시작한다
    _air_entry = !copter.ap.land_complete;

    _state = _air_entry ? State::FLIGHT_WAIT : State::NONE;
    _gcs_cmd.pending = false;
    _gcs_cmd.auto_pending = false;
    _auto_step = State::NONE;
    _auto_waiting = false;

    _target_loc = copter.current_loc;
    _target_heading_deg = degrees(ahrs.get_yaw());         

    home_init = false;

    _prearm_ready = false;

    _state_entered = _air_entry;            // 공중이면 state 5 진입 처리를 한 번 돈다
    _state_start_ms = AP_HAL::millis();

    _state_done = true;
    _action_retry_ms = 0;
    _was_armed = motors->armed();

    _armed_ms = AP_HAL::millis();
    _armed_prev = motors->armed();
    _takeoff_started = false;

    _hold_pos_neu_cm.zero();
    _track_pos_neu_cm.zero();
    _takeoff_target_neu_cm.zero();

    _status = TdcnStatus{};

    Arming = 1;

    if (_air_entry) {
        gcs().send_text(MAV_SEVERITY_INFO, "%s: air entry -> state %u", name(),
                        (unsigned)State::FLIGHT_WAIT);
    }

    return true;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 모드 이탈 */
void ModeTDCN::exit()
{
    // CLAW 가 몰던 중에 모드가 바뀌면 다음 모드가 쌓인 적분을 물려받지 않게 한다
    if (_status.claw_mask != 0 && !is_disarmed_or_landed()) {
        claw_handback(_status.claw_mask);
    }

    Arming = 0;

    _state = State::NONE;
    _auto_step = State::NONE;
    _status = TdcnStatus{};
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 메인 루프 (400Hz) */
void ModeTDCN::run()
{
    // 1. 기체 상태 점검
    check_vehicle_status();

    // 2. GCS 메시지 점검
    check_gcs_message();     

    // 3. state 별 처리
    switch (_state) 
    {
        case State::NONE:             make_safe_ground_handling();  break;

        case State::HANGAR_OPEN:      state_hangar_open();          break;      // 1  격납함 열기

        case State::TAKEOFF_WAIT:     state_takeoff_wait();         break;      // 2  이륙 대기

        case State::ARMED:            state_armed();                break;      // 3  ARMED

        case State::LAUNCH:           state_launch();               break;      // 4  이륙 사출

        case State::FLIGHT_WAIT:      state_flight_wait();          break;      // 5  비행 대기

        case State::TRACKING:         state_tracking();             break;      // 6  추종 비행

        case State::LANDING_WAIT:     state_landing_wait();         break;      // 7  착륙 대기

        case State::LANDING_SYNC:     state_landing_sync();         break;      // 8  착륙 동기

        case State::LANDING_STOW:     state_landing_stow();         break;      // 9  착륙 수납

        case State::DISARMED:         state_disarmed();             break;      // 10 DISARMED

        case State::HANGAR_CLOSE:     state_hangar_close();         break;      // 11 격납함 닫기

        case State::AUTO_TO_TRACKING: state_auto_to_tracking();     break;      // 12 1~6 자동 진행

        case State::AUTO_TO_CLOSE:    state_auto_to_close();        break;      // 13 6~11 자동 진행
    }

    // 4. 현재 상태 업데이트
    update_status();

    _state_entered = false;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 기체 상태 점검 */
void ModeTDCN::check_vehicle_status()
{
    const bool armed_now = motors->armed();
    if (armed_now && !_armed_prev) {
        _armed_ms = AP_HAL::millis();
    }
    _armed_prev = armed_now;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* GCS 명령 처리 */
MAV_RESULT ModeTDCN::GCS_command(const mavlink_command_int_t &packet)
{
    if (copter.flightmode != this) {
        return MAV_RESULT_DENIED;
    }

    if (!isfinite(packet.param1)) {
        return MAV_RESULT_DENIED;
    }
    const int32_t state_num = (int32_t)roundf(packet.param1);

    const State now_state = scenario_state();
    const State cur_state = _gcs_cmd.pending ? _gcs_cmd.state : now_state;
    const bool  cur_done  = (cur_state == now_state) ? _state_done : false;

    if (state_num == (int32_t)State::AUTO_TO_TRACKING ||
        state_num == (int32_t)State::AUTO_TO_CLOSE) {
        const State auto_state = (State)state_num;
        const bool ok = (auto_state == State::AUTO_TO_TRACKING)
                        ? (cur_state < State::TRACKING)
                        : (cur_state >= State::TRACKING && cur_state < State::HANGAR_CLOSE);
        if (!ok) {
            return MAV_RESULT_DENIED;
        }
        _gcs_cmd.auto_state = auto_state;
        _gcs_cmd.auto_pending = true;
        return MAV_RESULT_ACCEPTED;
    }

    if (state_num < (int32_t)State::HANGAR_OPEN ||
        state_num > (int32_t)State::HANGAR_CLOSE) {
        return MAV_RESULT_DENIED;
    }

    const State state = (State)state_num;

    if (!state_order_ok(cur_state, state)) {
        return MAV_RESULT_DENIED;
    }
    if (state != cur_state && !cur_done) {
        return MAV_RESULT_DENIED;
    }

    Location loc;
    if (state == State::TRACKING) {
        if (!isfinite(packet.z) || !isfinite(packet.param2)) {
            return MAV_RESULT_DENIED;
        }

        switch (packet.frame) {

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            loc.lat = packet.x;
            loc.lng = packet.y;
            break;

        case MAV_FRAME_LOCAL_NED:
            if (!ahrs.home_is_set()) {
                return MAV_RESULT_DENIED;   
            }
            loc = ahrs.get_home();
            loc.offset(packet.x * 0.01,         
                       packet.y * 0.01);       
            break;

        default:
            return MAV_RESULT_DENIED;
        }

        if (!ahrs.home_is_set()) {
            return MAV_RESULT_DENIED;
        }

        loc.set_alt_cm((int32_t)(packet.z * 100.0f),
                       Location::AltFrame::ABOVE_HOME);
    }

    _gcs_cmd.state = state;
    if (state == State::TRACKING) {
        _gcs_cmd.target_loc = loc;
        _gcs_cmd.target_heading_deg = packet.param2;
    }
    _gcs_cmd.pending = true;

    return MAV_RESULT_ACCEPTED;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* GCS 메시지 점검 */
void ModeTDCN::check_gcs_message()
{
    // --- GCS 가 보낸 state 명령 (1~11) ---
    if (_gcs_cmd.pending) {
        _gcs_cmd.pending = false;
        const State next = _gcs_cmd.state;

        if (next == State::TRACKING) {
            _target_loc = _gcs_cmd.target_loc;
            _target_heading_deg = _gcs_cmd.target_heading_deg;
        }

        if (is_auto(_state) && next == _auto_step) {
        } else {
            if (is_auto(_state)) {
                // 자동 진행 중 다른 번호 - 자동 진행을 멈추고 수동으로 넘겨받는다
                _state = _auto_step;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: auto stopped", name());
            }
            change_state(next);
        }
    }

    // --- GCS 가 보낸 자동 진행 명령 (12 / 13) ---
    if (_gcs_cmd.auto_pending) {
        _gcs_cmd.auto_pending = false;
        if (_state != _gcs_cmd.auto_state) {
            _auto_step    = scenario_state();
            _state        = _gcs_cmd.auto_state;
            _auto_waiting = false;
            gcs().send_text(MAV_SEVERITY_INFO, "%s: auto start -> state %u", name(),
                            (unsigned)(_state == State::AUTO_TO_TRACKING ? State::TRACKING
                                                                         : State::HANGAR_CLOSE));
        }
    }
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* state 전이 */
void ModeTDCN::change_state(State next)
{
    if (next == _state) {
        return;                
    }
    _state = next;
    _state_entered = true;
    _state_start_ms = AP_HAL::millis();

    _state_done = false;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 현재 상태 업데이트 */
void ModeTDCN::update_status()
{
    // --- 목표값: 지금 state 가 쫓는 목표 ---
    const bool flying = !is_disarmed_or_landed();
    bool valid = false;
    Vector3p target = _hold_pos_neu_cm;
    const State step = scenario_state();    // 자동 진행 중이면 그 안의 단계

    switch (step) {
    case State::NONE:
    case State::HANGAR_OPEN:
    case State::TAKEOFF_WAIT:
    case State::ARMED:                                  // 지상 대기 - 목표 없음
        break;
    case State::LAUNCH:
        valid = _takeoff_started;                       // 이륙 목표
        target = _takeoff_target_neu_cm;
        break;
    case State::FLIGHT_WAIT:
    case State::LANDING_WAIT:
    case State::LANDING_SYNC:                           // Z 는 state 8 이 동기 고도로 바꿔 둔다
    case State::DISARMED:                               // disarm 이 거부되어 공중일 때
        valid = flying;
        break;
    case State::TRACKING:
        valid = flying;
        target = _track_pos_neu_cm;                     // GCS 타겟
        break;
    case State::LANDING_STOW: {
        // 착륙 지점: 수평은 위치제어 목표, 고도는 home
        valid = flying;
        target = pos_control->get_pos_target_cm();
        Vector3f home_neu_cm;
        if (ahrs.get_home().get_vector_from_origin_NEU(home_neu_cm)) {
            target.z = home_neu_cm.z;
        }
        break;
    }
    case State::HANGAR_CLOSE:
    case State::AUTO_TO_TRACKING:                       // scenario_state() 는 이 둘을
    case State::AUTO_TO_CLOSE:                          // 돌려주지 않는다
        break;
    }

    _status.target_valid = valid;
    _status.target_pos_neu_cm = target;
    _status.target_heading_deg = (step == State::TRACKING)
                                 ? _target_heading_deg
                                 : degrees(attitude_control->get_att_target_euler_rad().z);

    // --- 현재값 ---
    _status.pos_neu_cm  = inertial_nav.get_position_neu_cm().topostype();
    _status.vel_neu_cms = inertial_nav.get_velocity_neu_cms();
    _status.euler_rad   = Vector3f(ahrs.get_roll(), ahrs.get_pitch(), ahrs.get_yaw());
    _status.gyro_rads   = ahrs.get_gyro();

    // --- 제어값: 5번에서 믹서에 넣을 CLAW 값과 대체할 축 ---
    const uint8_t mask = claw_output_mask();
    const uint8_t released = _status.claw_mask & ~mask;     // 지난 루프에 CLAW 가 몰다 놓은 축
    if (released != 0 && flying) {
        claw_handback(released);
    }
    _status.claw_mask = mask;

    // 스로틀 바이어스 = IBSC 호버(0.5) - 학습된 호버.  CLAW 가 스로틀을 잡기
    // 전까지 (state 5 포함) 따라가다 잡는 순간 고정
    static float thr_bias;
    if (!(mask & CLAW_THR)) {
        thr_bias = 0.5f - motors->get_throttle_hover();
    }

    if (step == State::TRACKING) {
        _status.claw_roll  = constrain_float(CLAW_Y.v_cmd.cmd_roll,  -1.0f, 1.0f);
        _status.claw_pitch = constrain_float(CLAW_Y.v_cmd.cmd_pitch, -1.0f, 1.0f);
        _status.claw_yaw   = constrain_float(CLAW_Y.v_cmd.cmd_yaw,   -1.0f, 1.0f);

        // -1 ~ +1  ->  0 ~ 1, 중심(0.5)을 학습된 호버로 옮긴다
        const float ch = constrain_float(CLAW_Y.v_cmd.cmd_height, -1.0f, 1.0f);
        _status.claw_throttle = constrain_float((ch + 1.0f) * 0.5f - thr_bias, 0.0f, 1.0f);
    } else {
        _status.claw_roll = _status.claw_pitch = _status.claw_yaw = 0.0f;
        _status.claw_throttle = 0.0f;
    }
}
// ---------------------------------------------------------------------------



bool ModeTDCN::state_order_ok(State from, State to)
{
    if (to == from) {
        return true;                                    // 같은 번호 재전송
    }
    return (uint8_t)to == (uint8_t)from + 1;            // 바로 다음 번호만
}

bool ModeTDCN::is_taking_off() const
{
    return (scenario_state() == State::LAUNCH) && !auto_takeoff.complete;
}

bool ModeTDCN::is_landing() const
{
    return scenario_state() == State::LANDING_STOW;
}

const char *ModeTDCN::state_name(State state)
{
    switch (state) {
    case State::NONE:           return "NONE";
    case State::HANGAR_OPEN:    return "HANGAR_OPEN";
    case State::TAKEOFF_WAIT:   return "TAKEOFF_WAIT";
    case State::ARMED:          return "ARMED";
    case State::LAUNCH:         return "LAUNCH";
    case State::FLIGHT_WAIT:    return "FLIGHT_WAIT";
    case State::TRACKING:       return "TRACKING";
    case State::LANDING_WAIT:   return "LANDING_WAIT";
    case State::LANDING_SYNC:   return "LANDING_SYNC";
    case State::LANDING_STOW:   return "LANDING_STOW";
    case State::DISARMED:       return "DISARMED";
    case State::HANGAR_CLOSE:   return "HANGAR_CLOSE";
    case State::AUTO_TO_TRACKING: return "AUTO_TO_TRACKING";
    case State::AUTO_TO_CLOSE:    return "AUTO_TO_CLOSE";
    }
    return "?";
}

// ---------------------------------------------------------------------------
/* CLAW 로 넘길 정보 갱신 */
void ModeTDCN::Update_Info_for_CLAW()
{
    // IMU - Gyro => CLAW: STV[3..5]
    const Vector3f &gyro = ahrs.get_gyro();
    CLAW_U.p = gyro.x;                           // roll  rate (rad/s, body F)
    CLAW_U.q = gyro.y;                           // pitch rate (rad/s, body R)
    CLAW_U.r = gyro.z;                           // yaw   rate (rad/s, body D)

    // IMU - Euler angle => CLAW: STV[6..8]
    CLAW_U.Roll                    = (real32_T)ahrs.get_roll();     // rad
    CLAW_U.Pitch                   = (real32_T)ahrs.get_pitch();    // rad
    CLAW_U.DR_heading_f.DR_heading =           ahrs.get_yaw();      // rad

    // EKF - Velocity => CLAW: XTV[0..2]
    const Vector3f &vel_neu_cms = inertial_nav.get_velocity_neu_cms();
    XTV[0] =  (double)vel_neu_cms.x * 0.01;       // North (m/s, N)
    XTV[1] =  (double)vel_neu_cms.y * 0.01;       // East  (m/s, E)
    XTV[2] = -(double)vel_neu_cms.z * 0.01;       // Down  (m/s, D)

    // Current Position
    const Location &loc = copter.current_loc;
    CLAW_U.Cur_Pos.x = (double)loc.lat * 1.0e-7;    // 위도 (deg)
    CLAW_U.Cur_Pos.y = (double)loc.lng * 1.0e-7;    // 경도 (deg)
    CLAW_U.Cur_Pos.z = (double)loc.alt * 0.01;      // 고도 (m, up)

    // Target Position (MAV_CMD_USER_1)
    CLAW_U.Dest_poti_i.x = (double)_target_loc.lat * 1.0e-7;   // 위도 (deg)
    CLAW_U.Dest_poti_i.y = (double)_target_loc.lng * 1.0e-7;   // 경도 (deg)
    // 고도는 Cur_Pos.z 와 같은 기준(ArduPilot home 기준 up m)으로 맞춘다.
    CLAW_U.Dest_poti_i.z = (double)_target_loc.alt * 0.01;     // 고도 (m, up)

    // Target Heading  (MAV_CMD_USER_1)
    CLAW_U.Ship_heading = radians(wrap_180(_target_heading_deg));   // (rad, 진북)
}
// ---------------------------------------------------------------------------


uint8_t ModeTDCN::claw_output_mask() const
{
    if (scenario_state() != State::TRACKING || !home_init ||
        !motors->armed() || is_disarmed_or_landed()) {
        return 0;
    }
    return (uint8_t)_claw_on_off.get() & CLAW_ALL;
}

void ModeTDCN::claw_handback(uint8_t released)
{
    // 스로틀: 고도 제어기 적분이 끝까지 쌓여 있다.  호버 스로틀에서 다시 시작한다
    // (init_z_controller 는 지금 스로틀로 적분을 잡으므로 먼저 호버로 맞춘다)
    if (released & CLAW_THR) {
        attitude_control->set_throttle_out(motors->get_throttle_hover(), true, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);
        pos_control->init_z_controller();
    }

    // 자세: 목표를 지금 자세로, 각속도 목표 0
    if (released & (CLAW_ROLL | CLAW_PITCH)) {
        attitude_control->reset_target_and_rate(true);
    } else if (released & CLAW_YAW) {
        attitude_control->reset_yaw_target_and_rate(true);
    }
    if (released & CLAW_ROLL) {
        attitude_control->get_rate_roll_pid().reset_I();
    }
    if (released & CLAW_PITCH) {
        attitude_control->get_rate_pitch_pid().reset_I();
    }
    if (released & CLAW_YAW) {
        attitude_control->get_rate_yaw_pid().reset_I();
    }

    // 수평: 속도 제어기 적분도 쌓여 있다.  돌고 있는 제어기는 init 이 적분을
    // 남기므로 따로 지운다
    if (released & (CLAW_ROLL | CLAW_PITCH)) {
        pos_control->init_xy_controller();
        pos_control->get_vel_xy_pid().reset_I();
    }
}

void ModeTDCN::output_to_motors()
{
    _status.ap_roll     = motors->get_roll()  + motors->get_roll_ff();
    _status.ap_pitch    = motors->get_pitch() + motors->get_pitch_ff();
    _status.ap_yaw      = motors->get_yaw()   + motors->get_yaw_ff();
    _status.ap_throttle = attitude_control->get_throttle_in();

    const uint8_t mask = _status.claw_mask;
    if (mask & CLAW_ROLL) {
        motors->set_roll(_status.claw_roll);
        motors->set_roll_ff(0.0f);
    }
    if (mask & CLAW_PITCH) {
        motors->set_pitch(_status.claw_pitch);
        motors->set_pitch_ff(0.0f);
    }
    if (mask & CLAW_YAW) {
        motors->set_yaw(_status.claw_yaw);
        motors->set_yaw_ff(0.0f);
    }
    if (mask & CLAW_THR) {
        motors->set_throttle(_status.claw_throttle);
    }

    Mode::output_to_motors();
}

void ModeTDCN::Run_CLAW()
{
    claw_gains.apply();       
    Update_Info_for_CLAW();
    CLAW_step();           
}

void ModeTDCN::state_hangar_open()      // 1 격납함 열기
{
    _state_done = true;
    make_safe_ground_handling();
}

void ModeTDCN::state_takeoff_wait()     // 2 이륙 대기 (prearm check)
{
    const bool ready = copter.ap.pre_arm_check;

    _state_done = ready;

    if (_state_entered || ready != _prearm_ready) {
        _prearm_ready = ready;
        gcs().send_text(ready ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                        "%s: prearm %s", name(), ready ? "OK" : "FAIL");
    }

    make_safe_ground_handling();
}

void ModeTDCN::state_armed()            // 3 ARMED
{
    _state_done = motors->armed();

    if (!_state_done) {
        const uint32_t now_ms = AP_HAL::millis();
        if (_state_entered || _was_armed || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        }
    }
    _was_armed = motors->armed();

    make_safe_ground_handling();
}

static const uint32_t _takeoff_settle_ms = 1000;    // 1 초

void ModeTDCN::state_launch()           // 4 이륙 사출
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_state_entered) {
        _takeoff_started = false;
    }

    if (!_state_done && !motors->armed()) {
        if (_state_entered || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        }
        if (!motors->armed()) {
            make_safe_ground_handling();
            return;                
        }
        _takeoff_started = false; 
    }

    if (!_takeoff_started && is_disarmed_or_landed() &&
        (now_ms - _armed_ms) < _takeoff_settle_ms) {
        make_safe_ground_handling();
        return;
    }

    if (!_takeoff_started) {
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        pos_control->set_max_speed_accel_z(-_takeoff_spd, _takeoff_spd, g.pilot_accel_z);
        pos_control->set_correction_speed_accel_z(-_takeoff_spd, _takeoff_spd, g.pilot_accel_z);
        pos_control->init_z_controller();

        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm((int32_t)_takeoff_alt, Location::AltFrame::ABOVE_HOME);
        int32_t alt_above_origin_cm;
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_above_origin_cm)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "%s: takeoff alt failed", name());
            return;
        }
        auto_takeoff.start((float)alt_above_origin_cm, false);

        _takeoff_target_neu_cm = inertial_nav.get_position_neu_cm().topostype();
        _takeoff_target_neu_cm.z = alt_above_origin_cm;

        copter.set_auto_armed(true);

        _takeoff_started = true;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: takeoff to %.1fm", name(), (double)(_takeoff_alt * 0.01f));
    }

    auto_takeoff.run();

    _state_done = auto_takeoff.complete;
}

void ModeTDCN::state_flight_wait()      // 5 비행 대기
{
    _state_done = true;

    if (_state_entered) {
        if (!auto_takeoff.get_completion_pos(_hold_pos_neu_cm)) {
            _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
        }

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        if (_air_entry) {
            // 공중 진입: home 바로 위 (0, 0, TKO_ALT), 헤딩 0 - 다시 이륙한 것처럼
            pos_control->init_xy_controller_stopping_point();
            pos_control->init_z_controller_stopping_point();
            _hold_pos_neu_cm = pos_control->get_pos_desired_cm();

            Vector3f home_neu_cm;
            if (ahrs.get_home().get_vector_from_origin_NEU(home_neu_cm)) {
                _hold_pos_neu_cm = Vector3p(home_neu_cm.x, home_neu_cm.y,
                                            home_neu_cm.z + _takeoff_alt);
            }

            // HOLD 에서 상대각으로 줘야 지금 헤딩부터 기본 선회율로 0 까지 돈다
            auto_yaw.set_fixed_yaw(wrap_180(-degrees(ahrs.get_yaw())), 0.0f, 0, true);
            _air_entry = false;
        }
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

void ModeTDCN::state_tracking()         // 6 추종 비행
{
    _state_done = true;

    if (_state_entered) {
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        _track_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    Run_CLAW();

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector3f target_neu_cm;
    if (_target_loc.get_vector_from_origin_NEU(target_neu_cm)) {
        _track_pos_neu_cm = target_neu_cm.topostype();
    }

    pos_control->input_pos_xyz(_track_pos_neu_cm, 0.0f, 0.0f);
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    auto_yaw.set_yaw_angle_rate(_target_heading_deg, 0.0f);

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());

    // CLAW 가 모는 축은 rate 적분을 비운다.  자세 목표는 3축 모두일 때만 지금
    // 자세로 맞춘다 (일부만이면 아두파일럿이 모는 축의 목표까지 지워진다)
    const uint8_t mask = claw_output_mask();
    if ((mask & CLAW_ATT) == CLAW_ATT) {
        attitude_control->reset_target_and_rate(false);
    }
    if (mask & CLAW_ROLL) {
        attitude_control->get_rate_roll_pid().reset_I();
    }
    if (mask & CLAW_PITCH) {
        attitude_control->get_rate_pitch_pid().reset_I();
    }
    if (mask & CLAW_YAW) {
        attitude_control->get_rate_yaw_pid().reset_I();
    }
}

void ModeTDCN::state_landing_wait()     // 7 착륙 대기
{
    _state_done = true;

    if (_state_entered) {
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

        pos_control->init_xy_controller_stopping_point();
        pos_control->init_z_controller_stopping_point();

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

void ModeTDCN::state_landing_sync()     // 8 착륙 동기
{
    if (_state_entered) {
        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(-_land_spd, wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(-_land_spd, wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    Location sync_loc = copter.current_loc;
    sync_loc.set_alt_cm((int32_t)_land_alt, Location::AltFrame::ABOVE_HOME);
    Vector3f sync_neu_cm;
    if (sync_loc.get_vector_from_origin_NEU(sync_neu_cm)) {
        _hold_pos_neu_cm.z = sync_neu_cm.z;   
    }

    _state_done = fabsf((float)copter.current_loc.alt - _land_alt) < 50.0f;

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

void ModeTDCN::state_landing_stow()     // 9 착륙 수납
{
    _state_done = copter.ap.land_complete;

    if (_state_entered) {
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),  wp_nav->get_wp_acceleration());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }

        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        copter.ap.land_repo_active = false;
        copter.ap.prec_land_active = false;

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    land_run_horiz_and_vert_control();
}

void ModeTDCN::state_disarmed()         // 10 DISARMED
{
    _state_done = !motors->armed();

    if (!_state_done) {
        const uint32_t now_ms = AP_HAL::millis();
        if (_state_entered || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.disarm(AP_Arming::Method::MAVLINK);
        }
    }

    if (_state_entered) {
        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

void ModeTDCN::state_hangar_close()     // 11 격납함 닫기
{
    _state_done = true;

    make_safe_ground_handling();
}

void ModeTDCN::state_auto_to_tracking() // 12 1~6 자동 진행
{
    auto_advance(State::TRACKING);

    switch (_auto_step) {
    case State::NONE:           make_safe_ground_handling(); break;
    case State::HANGAR_OPEN:    state_hangar_open();    break;  // 1
    case State::TAKEOFF_WAIT:   state_takeoff_wait();   break;  // 2
    case State::ARMED:          state_armed();          break;  // 3
    case State::LAUNCH:         state_launch();         break;  // 4
    case State::FLIGHT_WAIT:    state_flight_wait();    break;  // 5
    case State::TRACKING:       state_tracking();       break;  // 6 (도착한 루프)
    default:                                            break;
    }
}

void ModeTDCN::state_auto_to_close()    // 13 6~11 자동 진행
{
    auto_advance(State::HANGAR_CLOSE);

    switch (_auto_step) {
    case State::TRACKING:       state_tracking();       break;  // 6
    case State::LANDING_WAIT:   state_landing_wait();   break;  // 7
    case State::LANDING_SYNC:   state_landing_sync();   break;  // 8
    case State::LANDING_STOW:   state_landing_stow();   break;  // 9
    case State::DISARMED:       state_disarmed();       break;  // 10
    case State::HANGAR_CLOSE:   state_hangar_close();   break;  // 11 (도착한 루프)
    default:                                            break;
    }
}

void ModeTDCN::auto_advance(State end)
{
    const bool done = (_auto_step == State::NONE) || _state_done;
    if (!done) {
        _auto_waiting = false;
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (!_auto_waiting) {
        _auto_waiting = true;
        _auto_done_ms = now_ms;
        return;
    }
    const uint32_t dwell_ms = (uint32_t)(MAX(_auto_dwell.get(), 0.0f) * 1000.0f);
    if (now_ms - _auto_done_ms < dwell_ms) {
        return;
    }

    const State next = (State)((uint8_t)_auto_step + 1);
    if (next == State::TRACKING) {
        _target_loc = copter.current_loc;
        _target_heading_deg = degrees(ahrs.get_yaw());
    }
    _auto_step     = next;
    _auto_waiting  = false;
    _state_entered = true;
    _state_start_ms = now_ms;
    _state_done    = false;
    gcs().send_text(MAV_SEVERITY_INFO, "%s: auto state %u", name(), (unsigned)next);

    if (next == end) {
        _state = next;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: auto done (state %u)", name(), (unsigned)next);
    }
}
// ---------------------------------------------------------------------------


#endif  // MODE_TDCN_ENABLED