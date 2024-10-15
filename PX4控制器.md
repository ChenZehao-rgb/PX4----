### **1、位置控制器**

```c++
void PositionControl::_positionControl()
{
	// vel_sp_position = (_pos_sp - _pos) *  _gain_pos_p,均为Vector3f格式
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	/*  如果_vel_sp和vel_sp_position都不为NAN，_vel_sp += vel_sp_position；
   		如果_vel_sp为NAN，_vel_sp = vel_sp_position */
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	/*  如果vel_sp_position不为NAN，vel_sp_position += Vector3f()；
   		如果vel_sp_position为NAN，vel_sp_position = Vector3f() 
   		目的是保证没有NAN元素，用零元素覆盖NAN元素*/
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// 限制水平设定速度，优先保留vel_sp_position.xy()信息
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// 限制z方向速度输出
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}
```

![MC Position Control Diagram](https://gitee.com/ChenZehao-rgb/image/raw/master/MC%20Position%20Control%20Diagram.png)

### **2、速度控制器**

#### 计算微分_vel_dot

```c++
float BlockDerivative::update(float input)
{
	float output;
	if (_initialized) {
		output = _lowPass.update((input - getU()) / getDt());
	} // 数据已经有过一次有效更新，计算当前输入信号与前一次输入信号之差，并除以时间步长 getDt()
    else {
		_lowPass.update(0.0f); // 假设input没有太多变化，传递零值
		output = 0.0f; // 导数设置为0
		_initialized = true;// 初始化标志设置为true
	}
	setU(input);
	return output;
}
```

#### 设计低通滤波器：

$$
y[n] = a \cdot x[n] + (1 - a) \cdot y[n-1]
$$

其中：y[n] 是当前输出，x[n] 是当前输入，y[n-1] 是前一次的输出，a 是滤波器系数，决定了滤波器的截止频率

```c++
float BlockLowPass::update(float input)
{
	if (!PX4_ISFINITE(getState())) {
		setState(input);
	}
	float b = 2 * M_PI_F * getFCut() * getDt();
	float a = b / (1 + b);
    //低通滤波器离散时间传递函数
	setState(a * input + (1 - a)*getState());
	return getState();
}
```

#### **加速度控制**

```c++
void PositionControl::_accelerationControl()
{
	// 通过 _acc_sp 的前两个分量取负值，并将第三个分量设为 CONSTANTS_ONE_G （标准重力加速度），生成一个新的三维向量 body_z
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	// 对 body_z 进行倾斜限制,里面由向量点积求角度的计算公式有问题，但是对整体计算没有太大影响，可以起到限位作用
    ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// 相当于 F= a( f/g ) - f , f~mg, 要注意_hover_thrust是如何传递的
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// collective_thrust 投影到机体姿态上，确保推理方向与机体姿态方向一致
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
    // 限制在最小推力
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
    // 计算 _thr_sp
	_thr_sp = body_z * collective_thrust;
}
```

#### 速度控制器代码

```c++
void PositionControl::_velocityControl(const float dt)
{
	// 限制垂直方向速度积分
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
    // 运行PID控制器，_vel_dot通过 PositionControl::setState获得，即为当前加速度
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	/*  如果_acc_sp和acc_sp_velocity都不为NAN，_acc_sp += acc_sp_velocity；
   		如果_acc_sp为NAN，_acc_sp = acc_sp_velocity */
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
    
    // _acc_sp ——> _thr_sp
	_accelerationControl();

	// 垂直方向上抗风偏，防止系统在推力达到限制时不会积累过多误差
    // 假如 _thr_sp(2) 为-0.5，_lim_thr_min 为1， 升力绝对值小于 _lim_thr_min， 速度误差为正，即正在下降，归零使不再下降
    // 假如 _thr_sp(2) 为-6，_lim_thr_max 为5， 升力绝对值大于 _lim_thr_max， 速度误差为负，即正在上升，归零使不再上升
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	/* 优先保持垂推力，确保水平推力在合理范围内*/
    
	const Vector2f thrust_sp_xy(_thr_sp); // explicit Vector2(const Vector3 &other)
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// 保持水平推力余量的同时，确定剩余的垂直推力
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// 限制最大垂直推力
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// 确定优先控制垂直推力后剩余的水平推力
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// 得到水平推力
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
    // 实际产生的加速度，相当于 a = F * (g / f) = F * (1/m) , 水平方向抗风偏
	const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0); // 抗风偏增益

	// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
	// The ARW loop needs to run if the signal is saturated only.
	const Vector2f acc_sp_xy = _acc_sp.xy(); //提取水平分量
	// 确定受限的水平加速度
    const Vector2f acc_limited_xy = (acc_sp_xy.norm_squared() > acc_sp_xy_produced.norm_squared())
					? acc_sp_xy_produced
					: acc_sp_xy;
	// 在信号饱和时，调整速度误差以防止积分器累积过多的误差
    vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_limited_xy);

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// 更新速度控制积分项
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}
```

![MC Velocity Control Diagram](https://gitee.com/ChenZehao-rgb/image/raw/master/mc_velocity_diagram.D60AUT18.png)

### **3、位置控制整体逻辑**

```c++
void MulticopterPositionControl::Run()
{
    /* 开始时检查是否应该退出 */
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms); //任务调度，周期100ms
	parameters_update(false); //更新参数，参数为false，保证只有_parameter_update_sub更新时才会执行
	perf_begin(_cycle_perf); //性能计数，测量代码执行的时间
    
	vehicle_local_position_s vehicle_local_position //定义本地位置结构体
	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;
		// dt的数量级，2ms—40ms?,和任务调度周期有什么关系?
		// set _dt in controllib Block for BlockDerivative
		setDt(dt);

		if (_vehicle_control_mode_sub.updated()) {
			//位置控制状态传递
            const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;
			//如果更新_vehicle_control_mode成功
            if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
                // 之前的位置控制模式未启用，而现在启用了位置控制模式，表示位置控制模式刚刚启用
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					//更新时间戳，记录位置模式启动时间
                    _time_position_control_enabled = _vehicle_control_mode.timestamp;
				} 
                // 位置模式刚刚禁用
                else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = PositionControl::empty_trajectory_setpoint;
				}
			}
		}
		//更新着陆状态
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);
		//更新悬停推力估计值
		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;
			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}
		//使用列表初始化语法将 set_vehicle_states(vehicle_local_position) 的返回值赋给 states 对象
		PositionControlStates states{set_vehicle_states(vehicle_local_position)};

		// if a goto setpoint available this publishes a trajectory setpoint to go there
		if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample,
						   _vehicle_control_mode.flag_multicopter_position_control_enabled)) {
			_goto_control.update(dt, states.position, states.yaw);
		}
		
        // 从更新中获取轨迹设定点
		_trajectory_setpoint_sub.update(&_setpoint);
		// 调整轨迹设定点，确保其与EKF重置后的状态一致，首先判断设定点时间戳与当前时间戳，然后判断各轴重置计数值是否不同，随后调整设定点速度/位置/航向
		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);
		// 检查是否使用多旋翼位置控制模式
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			// set failsafe setpoint if there hasn't been a new trajectory setpoint since position control started
			// 如果轨迹设定点的时间戳早于位置控制模式启用的时间，并且当前时间戳晚于位置控制模式启用的时间
            if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {
				// 在没有新的轨迹设定点时，使用故障保护设定点进行控制
                // states.velocity为有限值，failsafe_setpoint.velocity设置为0值，无人机停止
                // states.velocity无限，failsafe_setpoint.acceleration水平方向设为0，垂直方向速度设为降落速度
				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}
			// 检查是否启用offboard模式
			if (_vehicle_control_mode.flag_control_offboard_enabled) {
				// want_takeoff 变量为真表示飞行器已解锁（armed），并且当前时间戳小于设定点时间戳加 1 秒
				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (vehicle_local_position.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
					   && (_setpoint.velocity[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			bool skip_takeoff = _param_com_throw_en.get();
			// handle smooth takeoff，更新起飞状态
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, skip_takeoff, vehicle_local_position.timestamp_sample);

			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);
			// 如果尚未起飞，设置悬停推力
			if (!flying) {
				_control.setHoverThrust(_param_mpc_thr_hover.get());
			}

			// make sure takeoff ramp is not amended by acceleration feed-forward
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}
			// 尚未起飞或在飞行中接触地面，代码会重置设定点并防止积分器累积
			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				_setpoint = PositionControl::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

				// prevent any integrator windup
				_control.resetIntegral();
			}

			// limit tilt during takeoff ramupup
            // 如果飞行器尚未进入飞行状态（TakeoffState::flight），则使用地面倾斜角度限制参数 _param_mpc_tiltmax_lnd；否则，使用空中倾斜角度限制参数 _param_mpc_tiltmax_air
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
            // tilt_limit_deg ->弧度制，通过 _tilt_limit_slew_rate 更新，传到PositionControl::_lim_tilt
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));
			// 更新垂直速度限制
			const float speed_up = _takeoff.updateRamp(dt,
					       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
			// 将最小推力和最大推力作为推力限制
            _control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());
			// 设置水平速度最大值
			float max_speed_xy = _param_mpc_xy_vel_max.get();
			// 保证水平速度不超过飞行器物理限制
			if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
				max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
			}
			// 设置飞行器速度限制
			_control.setVelocityLimits(
				max_speed_xy,
				math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
				math::max(speed_down, 0.f));
			// 保证控制器使用最新的设定点计算飞行器控制输出
			_control.setInputSetpoint(_setpoint);

			// update states,需满足一系列条件确定是否需要更新垂直速度
			if (!PX4_ISFINITE(_setpoint.position[2])
			    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				// 计算0-1的权重，_setpoint.velocity[2] / _param_mpc_land_speed.get()
                float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
				// 垂直方向速度插值计算，高度导数(垂直速度估计值) * weighting + 垂直速度 * (1.f - weighting)
                states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
			}
			// 传递更新后的状态
			_control.setState(states);

			// Run position control
			if (!_control.update(dt)) {
				// Failsafe 位置控制更新失败，执行故障保护措施
				_vehicle_constraints = {0, NAN, NAN, false, {}}; // reset constraints
				// 生成一个FailsafeSetpoint，传递给控制器
				_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
				// 速度限制
                _control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());
				// 再次运行位置控制
                _control.update(dt);
			}

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skipped with non-altitude-controlled modes
			// 更新飞行状态
            _takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    vehicle_local_position.timestamp_sample);
		}

		// Publish takeoff status
        // 获取当前起飞状态并转换为uint8_t类型
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());
		// 检查当前起飞状态和倾斜限制是否发生了变化
		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}
	// 结束性能计数器的计时过程
	perf_end(_cycle_perf);
}
```



### **4、角度控制器**



### **5、角速率控制器**
