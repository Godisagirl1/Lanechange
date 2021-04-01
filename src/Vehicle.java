public abstract class Vehicle extends CommunicationEntity{
	public String toString() { return "[Vehicle at "+Simulator.getSingletonClock().getCurrentTick()+" tick]"; }
	public String toStringLong() {
		return "T"/*'T' for trajectory logging*/+this.toString()+": node = "+this.getNodeString()
			+" kineticState = "+this.kineticState.toString()
			+" current driving mode started at "+this.startTimeOfCurrentDrivingModeTick
			+" (tick) and kineticState last updated at "+this.lastKineticStateUpdateTimeTick+" (tick).";
	}

	///public final static double d1 = 174.8862 ;
	///public final static double dac = 152.8121;
	///public final static double d2 = 129.4281;
	///public final static double d3 = 175.1140;
	///public final static double d4 = 130.3711;
	///public final static double d5 = 140.1390;
	///public final static double d6 = 125.2081;
	
	public enum DrivingMode{
		STOPPED,
		FORWARD_CONSTANT_SPEED,
		FORWARD_ACCELERATE,
		FORWARD_DECELERATE,
		LANE_CHANGE,
		/**
		 * Under this driving mode, this.updateKineticStateWithoutDrivingModeChange() will not change this.kineticState.
		 * Instead, the user needs to call this.updateKineticStateWithoutDrivingModeChange(9 parameters ...)
		 * manually to explicitly change this.kineticState when needed.
		 */
		MANUALLY_SET_KINETIC_STATE 
	}
	public DrivingMode getDrivingMode() { return this.kineticState.drivingMode; }
	public String getDrivingModeString() {
		switch(this.kineticState.drivingMode) {
		case STOPPED: return "STOPPED"; 
		case FORWARD_CONSTANT_SPEED: return "FORWARD_CONSTANT_SPEED";
		case FORWARD_ACCELERATE: return "FORWARD_ACCELERATE";
		case FORWARD_DECELERATE: return "FORWARD_DECELERATE";
		case LANE_CHANGE: return "LANE_CHANGE";
		case MANUALLY_SET_KINETIC_STATE: return "MANUALLY_SET_KINETIC_STATE";
		}
		throw new RuntimeException(this.toString()+": unrecognized driving mode = "+this.kineticState.drivingMode);
	}
	
	/** unit: m */
	public double get_X() { return this.kineticState.X; }
	/** unit: m */
	public double get_Y() { return this.kineticState.Y; }
	/** unit: rad */
	public double get_psi() { return this.kineticState.psi; }
	
	/** unit: m/s */
	public double get_dot_X() { return this.kineticState.dot_X; }
	/** unit: m/s */
	public double get_dot_Y() { return this.kineticState.dot_Y; }
	/** unit: rad/s */
	public double get_dot_psi() { return this.kineticState.dot_psi; }
	/** unit: m/s */
	public double getSpeed() {
		return Math.sqrt(this.kineticState.dot_X * this.kineticState.dot_X + this.kineticState.dot_Y * this.kineticState.dot_Y); 
	}
	
	/** unit: m/s^2 */
	public double get_ddot_X() { return this.kineticState.ddot_X; }
	/** unit: m/s^2 */
	public double get_ddot_Y() { return this.kineticState.ddot_Y; }
	/** unit: rad/s^2 */
	public double get_ddot_psi() { return this.kineticState.ddot_psi; }
		
	/** unit: m/s */
	private double v_lim;
	public double get_v_lim() { return this.v_lim; }
	/** unit: m/s */
	private double v_low;
	public double get_v_low() { return this.v_low; }

	
	//Vehicle mechanics related parameters
	/**
	 * $m$, mass of the vehicle, unit: kg
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). 
	 * @see Wu Ren Jia Shi Che Liang Mo Xing content pp112
	 * 1723kg is the original value in Wu Ren Jia Shi Che Liang Mo Xing.
	 * 970kg is the value used in Che Liang Gong Cheng Fang Zhen Yu Fen Xi for acceleration analysis. 
	 * For consistency, we unified to 970kg.
	 */
	///private double m = 1723; 
	private double m = 970;
	/** $g$, gravitational acceleration, unit: m/s$^2$ */
	private double g = 9.8;
	/**
	 * $Iz$, yaw moment of inertia, unit: [kg.m^2]
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). 
	 * @see Wu Ren Jia Shi Che Liang Mo Xing content pp112, I = 
	 * 4175 is the yaw rotational momentum used in Vehicle Dynamics and Control (2ed).
	 * For consistency of m (@see m), we scale it down to 2350.
	 */
	///private double I_z = 4175; 
	private double I_z = 2350;
	/**
	 * Distance between front axle and cog unit: m
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). 
	 * @see Wu Ren Jia Shi Che Liang Mo Xing content pp112, a
	 */
	private double l_f = 1.232;  
	/**
	 * Distance between rear axle and cog unit: m
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31).
	 * @see Weu Ren Jia Shi Che Liang Mo Xing conent pp112, b 
	 */
	private double l_r = 1.468;
	/**
	 * lateral tire-stiffness front wheels
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). 
	 * @see Wu Ren Jia Shi Che Liang Mo Xing content pp112, Ccf
	 */
	private double C_alpha_f = 66900;
	/**
	 * lateral tire-stiffness rear wheels
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). 
	 * @see Wu Ren Jia Shi Che Liang Mo Xing content pp112, Ccr
	 */
	private double C_alpha_r = 62700; 
	/**
	 * slope for the triangular wave of $\delta$ when the vehicle is driving at $v_{lim}$, unit: rad/s
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). 
	 */
	private double k_v_lim = 0.50865 / 180.0 * Math.PI;
	/**
	 * slope for the triangular wave of $\delta$ when the vehicle is driving at $v_{low}$, unit: rad/s
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). 
	 */
	private double k_v_low = 0.7701 / 180.0 * Math.PI;
	/**
	 * quaterPeriod for the triangular wave of $\delta$, unit: tick
	 */
	private long quarterPeriodTick = 100;

	/** $dt$, increment of time change, unit: s. 
	protected double dt = 0.01;*/
	/** $r$, wheel radius, unit: m */
	private double r = 0.272;
	/** $\eta$, the powertrain efficiency. */
	private double eta = 0.9;
	/** $i_0$, the final drive ratio. */
	private double i_0 = 4.388;
	/** minimum engine angular speed, unit: round/minute. */
	private int n_min = 800;
	/** maximum engine angular speed, unit: round/minute. */
	private int n_max = 5400;
	/** $i_g$, gear ratio for 5 gears. */
	private double[] i_g = {3.416, 1.894, 1.280, 0.914, 0.757};
	/** the minimum speed for each gear, unit: km/h */
	private double[] u_min;
	/** the maximum speed for each gear, unit: km/h */
	private double[] u_max;
	/** $\delta$ in Qi Che Gong Cheng Fang Zhen Yu Fen Xi, $\lambda$ in our paper, rotational inertia constant.  */
	private double[] lambda;
	/** $C_D A$, drag coefficient (0.3) times vehicle frontal area (2.3 m$^2$). */
	private double C_D_A = 0.69;
	/**
	 * $\beta$, coefficient on engine angular speed drop when changing gears. 
	 * @see Che Liang Gong Cheng Fang Zhen Yu Fen Xi.
	 * This variable is not used in this program, as we require beta == 1, so that acc is monotonically increasing.
	 */
	private final double beta = 1;


	/**
	 * @param u_a, the speed, unit: km/h.
	 * @return $n$, the engine angular speed, unit: round/minute. 
	 * See page 6-7 of Che Liang Gong Cheng Fang Zhen Yu Fen Xi.
	 */
	private double get_n(double u_a, int currentGearIndex) {
		double n = i_g[currentGearIndex] * i_0 * u_a / r / 0.377;
		return n;
	}
	/**
	 * @param $n$, the engine angular speed, unit: round/minute.
	 * @return $T_{tq}$, the engine torque output, unit: Nm.
	 * See page 6-7 of Che Liang Gong Cheng Fang Zhen Yu Fen Xi.
	 */
	private double get_T_tq(double n) {
		double T_tq = 54.8179 + 2.2441 * (n / 100.0) - 4.8003 * (n / 1000.0) * (n / 1000.0) + 2.815e-10 * n * n * n;
		return T_tq;
	}
	/**
	 * @param $n$, the engine angular speed, unit: round/minute.
	 * @param currentGearIndex, current gear index.
	 * @return $F_t$, the traction force, unit: N.
	 * See page 6-7 of Che Liang Gong Cheng Fang Zhen Yu Fen Xi.
	 */
	private double get_F_t(double n, int currentGearIndex) {
		double T_tq = get_T_tq(n);
		double F_t = T_tq * i_0 * eta * i_g[currentGearIndex] / r;
		return F_t;
	}
	/**
	 * @param u_a, the speed, unit: km/h.
	 * @return  $F_r$, the rolling resistance force, unit: N.
	 * See page 6-7 item of $F_f$ of Che Liang Gong Cheng Fang Zhen Yu Fen Xi.
	 */
	private double get_F_r(double u_a) {
		double centUa = u_a / 100.0;
		double f = 0.009 + 0.002 * (centUa) + 0.0003 * (centUa * centUa * centUa * centUa);
		double F_r = m * g * f;
		return F_r;
	}
	/**
	 * @param u_a, the speed, unit: km/h.
	 * @return $F_w$, the aerodynamic drag force, unit: N. 
	 * See page 6-7 of Che Liang Gong Cheng Fang Zhen Yu Fen Xi.
	 */
	private double get_F_w(double u_a) {
		double F_w = C_D_A * u_a * u_a / 21.15;
		return F_w;
	}
	/**
	 * @param F_t, tract force, unit: N;
	 * @param F_r, rolling resistance force, unit: N;
	 * @param F_w, aerodynamic drag force, unit: N;
	 * @param gearIndex.
	 * @return $a$, the corresponding acceleration, unit: m/s$^2$. 
	 * See page 14 line for j1 and page 13 line for delta in
	 * Che Liang Gong Cheng Fang Zhen Yu Fen Xi.
	 */
	private double get_a(double F_t, double F_r, double F_w, int gearIndex) {
		double a = (F_t - F_r - F_w) / (lambda[gearIndex] * m);
		return a;
	}
	/**
	 * Presumption: the vehicle is truly accelerating (not constant velocity, not decelerating), 
	 * i.e. this.kineticState.drivingMode == DrivingMode.FORWARD_ACCELERATE.
	 * @param velocityMs velocity along the x-axis, i.e. this.kineticState.dot_x, unit: m/s
	 * @param gearIndex current gear index.
	 * @return the acceleration, unit: m/s$^2$, given currently the vehicle is accelerating.
	 */
	protected double get_a(double velocityMs, int gearIndex) {
		double velocityKmh = Vehicle.msToKmh(velocityMs); //unit: km/h. 
		if (gearIndex < 0 || gearIndex >= this.u_max.length)
			throw new RuntimeException(this.toString()+": gearIndex = "+gearIndex);
		if (velocityKmh < this.u_min[gearIndex] || velocityKmh > this.u_max[gearIndex])
			throw new RuntimeException(this.toString()+": velocityMs = "+velocityMs+" (m/s) = "
					+velocityKmh+" (km/h) not in u_min["+gearIndex+"] = "+this.u_min[gearIndex]
					+"u_max["+gearIndex+"] = "+this.u_max[gearIndex]+".");
		double n = this.get_n(velocityKmh, gearIndex); //rotation speed, unit: round/minute
		double F_t = this.get_F_t(n, gearIndex); //tract force, unit: N
		double F_r = this.get_F_r(velocityKmh); //rotational friction force, unit: N
		double F_w = this.get_F_w(velocityKmh); //wind drag, unit: N
		double acceleration = this.get_a(F_t, F_r, F_w, gearIndex);
		return acceleration;
	}

	public static double kmhToMs(double velocityKmh) {
		return velocityKmh / 3.6;
	}
	public static double msToKmh(double velocityMs) {
		return velocityMs * 3.6;
	}
	
	/**
	 * @param velocityMs velocity along the x-axis, i.e. this.kineticState.dot_x, unit: m/s
	 * @return the corresponding gear index. Supposedly, the minimum gear index whose range can cover velocityMs.
	 */
	public int calculateGearIndexUnderForwardAccelerateDrivingMode(double velocityMs) {
		if (0 <= velocityMs && velocityMs <= epsilon) return 0;
		double velocityKmh = msToKmh(velocityMs);
		if (velocityKmh < this.u_min[0]) 
			throw new RuntimeException(this.toString()+": 0 < velocityKmh = "+velocityKmh+"Kmh < u_min[0], cannot getGearIndex.");
		int gearIndex = 0;
		while (velocityKmh > this.u_max[gearIndex]) {
			gearIndex++;
			if (gearIndex >= this.u_max.length)
				throw new RuntimeException(this.toString()+": velocityMs = "+velocityMs+" exceeded max gear index capability, gearIndex = "+gearIndex);
		}
		return gearIndex;
	}


	/** 
	 * @see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). Note $\dot{X} = V_x \cos \psi - V_y \sin \psi; \dot{Y} = V_x \sin \psi + V_y \cos \psi$; V_x is dot_x; V_y is dot_y.
	 *     Also note pp34, just above Eq(2.38), talks about $V_x$ (i.e. dot_x) can be considered as constant, and $R$ is given by the road shape.
	 * @param k    $\delta$ change speed (unit: rad/s), where $\delta$ (unit: rad) means the steering angle. 
	 * @param quarterPeriodTick    During lane change, the vehicle R's $\delta$ changes as per a triangle wave consisting of four quarter periods. 
	 *     Quarter 0: $\delta$ changes from 0 rad to (k * quarterPeriodTick * secPerTick) rad; 
	 *     Quarter 1: $\delta$ changes from (k * quarterPeriodTick * secPerTick) rad to 0 rad;
	 *     Quarter 2: $\delta$ changes from 0 rad to -(k * quarterPeriodTick * secPerTick) rad;
	 *     Quarter 3: $\delta$ changes from -(k * quarterPeriodTick * secPerTick) rad to 0 rad.
	 * @return    front wheel steering angle, $\delta$, unit:rad
	 */
	private double get_delta(double k, long quarterPeriodTick, double durationOfATick, long startTimeTick, long currentTimeTick) {
		long durationTick = currentTimeTick - startTimeTick;
		if (durationTick < 0) throw new RuntimeException("startTimeTick < currentTimeTick");
		if (durationTick >=0 && durationTick < quarterPeriodTick){
			return k * durationTick * durationOfATick;
		}else if (durationTick >= quarterPeriodTick && durationTick < 2 * quarterPeriodTick){
			return k * quarterPeriodTick * durationOfATick - k * (durationTick - quarterPeriodTick) * durationOfATick;
		}else if (durationTick >= 2 * quarterPeriodTick  && durationTick < 3 * quarterPeriodTick){
			return - k * (durationTick - 2 * quarterPeriodTick) * durationOfATick;
		}else if (durationTick >= 3 * quarterPeriodTick  && durationTick < 4 * quarterPeriodTick){
			return - k * quarterPeriodTick * durationOfATick + k * (durationTick - 3 * quarterPeriodTick) * durationOfATick;
		}else{
			return 0;
		}
	}

	/**
	 * @see Vehicle Dynamics and Control Ed2 pp30 Eq(2.31) $A$ represents the 4x4 matrix on the right hand side.
	 * @param V_x $V_x$ (i.e. dot_x) in Vehicle Dynamics and Control Ed2 pp30 Eq(2.31). 
	 * @return the 2nd row 2nd column element of $A$ (index starts from 1)
	 */
	private double get_A_2_2(double V_x) {
		double result = - (2 * C_alpha_f + 2 * C_alpha_r) /(m * V_x);
		return result;
	}
	/**
	 * @see Vehicle Dynamics and Control Ed2 pp30 Eq(2.31) $A$ represents the 4x4 matrix on the right hand side.
	 * @param V_x $V_x$ (i.e. dot_x) in Vehicle Dynamics and Control Ed2 pp30 Eq(2.31).
	 * @return the 2nd row 4th column element of $A$ (index starts from 1)
	 */
	private double get_A_2_4(double V_x) {
		double result = - V_x - (2 * C_alpha_f * l_f - 2 * C_alpha_r * l_r) /(m * V_x);
		return result;
	}
	/**
	 * @see Vehicle Dynamics and Control Ed2 pp30 Eq(2.31) $B$ represents the 4x1 matrix on the right hand side.
	 * @return the 2nd row 1st column element of $B$ (index starts from 1)
	 */
	private double get_B_2_1() {
		double result = 2 * C_alpha_f / m;
		return result;
	}
	/**
	 * @see Vehicle Dynamics and Control Ed2 pp30 Eq(2.31) $A$ represents the 4x4 matrix on the right hand side.
	 * @param V_x $V_x$ (i.e. dot_x) in Vehicle Dynamics and Control Ed2 pp30 Eq(2.31).
	 * @return the 4th row 2nd column element of $A$ (index starts from 1)
	 */
	private double get_A_4_2(double V_x) {
		double result = - (2 * C_alpha_f * l_f - 2 * C_alpha_r * l_r) /(I_z * V_x); 
		return result;
	}
	/**
	 * @see Vehicle Dynamics and Control Ed2 pp30 Eq(2.31) $A$ represents the 4x4 matrix on the right hand side.
	 * @param V_x $V_x$ (i.e. dot_x) in Vehicle Dynamics and Control Ed2 pp30 Eq(2.31).
	 * @return the 4th row 4th column element of $A$ (index starts from 1)
	 */
	private double get_A_4_4(double V_x) {
		double result = - (2 * C_alpha_f * l_f * l_f + 2 * C_alpha_r * l_r * l_r) /(I_z * V_x); 
		return result;
	}
	/**
	 * @see Vehicle Dynamics and Control Ed2 pp30 Eq(2.31) $B$ represents the 4x1 matrix on the right hand side.
	 * @return the 4th row 1st column element of $B$ (index starts from 1)
	 */
	private double get_B_4_1() {
		double result = 2 * C_alpha_f * l_f / I_z;
		return result;
	}
	
	private long startTimeOfCurrentDrivingModeTick = 0;
	private long lastKineticStateUpdateTimeTick = 0;
	
	private class KineticState implements java.lang.Cloneable{
		private DrivingMode drivingMode = DrivingMode.STOPPED;
		private double X;
		private double Y;
		private double psi;
		private double x;
		private double y;
		private double dot_X;
		private double dot_Y;
		private double dot_psi;
		/** i.e. $V_x$ in Vehicle Dynamics and Control Ed2 pp30 Eq(2.31). */
		private double dot_x; 
		/** i.e. $V_y$ in Vehicle Dynamics and Control Ed2 pp30 Eq(2.31). */
		private double dot_y;
		private double ddot_X;
		private double ddot_Y;
		private double ddot_psi;
		private double ddot_x;
		private double ddot_y;
		
		public Object clone() {
			KineticState c = new KineticState();
			c.drivingMode = this.drivingMode;
			c.X = this.X;
			c.Y = this.Y;
			c.psi = this.psi;
			c.x = this.x;
			c.y = this.y;
			c.dot_X = this.dot_X;
			c.dot_Y = this.dot_Y;
			c.dot_psi = this.dot_psi;
			c.dot_x = this.dot_x;
			c.dot_y = this.dot_y;					
			c.ddot_X = this.ddot_X;
			c.ddot_Y = this.ddot_Y;
			c.ddot_psi = this.ddot_psi;
			c.ddot_x = this.ddot_x;
			c.ddot_y = this.ddot_y;
			return c;
		}
		
		public String toString() {
			return "( drivingMode = "+Vehicle.this.getDrivingModeString()
				+" X = "+this.X+" Y = "+this.Y+" psi = "+this.psi+" x = "+this.x+" y = "+this.y
				+" dot_X = "+this.dot_X+" dot_Y = "+this.dot_Y+" dot_psi = "+this.dot_psi+" dot_x = "+this.dot_x+" dot_y = "+this.dot_y		
				+" ddot_X = "+this.ddot_X+" ddot_Y = "+this.ddot_Y+" ddot_psi = "+this.ddot_psi+" ddot_x = "+this.ddot_x+" ddot_y = "+this.ddot_y+" )";
		}
		
		/**
		 * @see Vehicle Dynamics and Control Ed2 pp30 Eq(2.31) for variable meanings. Note dot_x is the same as $V_x$, and dot_y is the same as $V_y$.
		 */
		//private KineticState(DrivingMode drivingMode, double X, double Y, double psi, double dot_X, double dot_Y, double dot_psi, double ddot_X, double ddot_Y, double ddot_psi) {
		//	setNewDrivingModeAndKineticState(drivingMode, X, Y, psi, dot_X, dot_Y, dot_psi, ddot_X, ddot_Y, ddot_psi);
		//}
		//private KineticState() {
		//	this(DrivingMode.STOPPED, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		//}
	}
	/** My current kinetic state */
	private KineticState kineticState = new KineticState();
	
	/**
	 * Manually set the values on X, Y, psi, x, y, dot_X, dot_Y, dot_psi, dot_x, dot_y, ddot_X, ddot_Y, ddot_psi, ddot_x, ddot_y of
	 * this.kineticState.
	 * 
	 * this.kineticState.drivingMode is not changed.
	 * 
	 * @assume Note you can call this method when this.kineticState.drivingMode == MANUALLY_SET_KINETIC_STATE.
	 */
	protected synchronized void updateKineticStateWithoutDrivingModeChange(double new_X, double new_Y, double new_psi, 
			double new_dot_X, double new_dot_Y, double new_dot_psi, 
			double new_ddot_X, double new_ddot_Y, double new_ddot_psi) {
		this.kineticState.X = new_X;
		this.kineticState.Y = new_Y;
		this.kineticState.psi = new_psi;
		this.kineticState.x = 0; //at any time t, x-y coordinate (t)'s original point is fixed at the center of gravity of the vehicle at (t).
		this.kineticState.y = 0; //at any time t, x-y coordinate (t)'s original point is fixed at the center of gravity of the vehicle at (t).
		this.kineticState.dot_X = new_dot_X;
		this.kineticState.dot_Y = new_dot_Y;
		double new_speed = Math.sqrt(new_dot_X * new_dot_X + new_dot_Y * new_dot_Y);
		if (Math.abs(new_speed * Math.cos(new_psi) - new_dot_X) > epsilon)
			throw new RuntimeException(this.toString()+" (new_speed = "+new_speed+") * cos(new_psi = "+new_psi+") != (new_dot_X = "+new_dot_X+")");
		if (Math.abs(new_speed * Math.sin(new_psi) - new_dot_Y) > epsilon)
			throw new RuntimeException(this.toString()+" (new_speed = "+new_speed+") * sin(new_psi = "+new_psi+") != (new_dot_Y = "+new_dot_Y+")");		
		this.kineticState.dot_psi = new_dot_psi;
		this.kineticState.dot_x = new_dot_X * Math.cos(-new_psi) - new_dot_Y * Math.sin(-new_psi);
		this.kineticState.dot_y = new_dot_X * Math.sin(-new_psi) + new_dot_Y * Math.cos(-new_psi);
		this.kineticState.ddot_X = new_ddot_X;
		this.kineticState.ddot_Y = new_ddot_Y;
		//double new_acceleration = Math.sqrt(new_ddot_X * new_ddot_X + new_ddot_Y * new_ddot_Y);
		this.kineticState.ddot_psi = new_ddot_psi;
		this.kineticState.ddot_x = new_ddot_X * Math.cos(-new_psi) + new_dot_X * new_dot_psi * Math.sin(-new_psi) - new_ddot_Y * Math.sin(-new_psi) + new_dot_Y *new_dot_psi* Math.cos(-new_psi);
		this.kineticState.ddot_y = new_ddot_X * Math.sin(-new_psi) - new_dot_X * new_dot_psi * Math.cos(-new_psi) + new_ddot_Y * Math.cos(-new_psi) + new_dot_Y *new_dot_psi* Math.sin(-new_psi);
		
		double tmp;
		switch (this.kineticState.drivingMode) {
		case STOPPED:
			//X can be any value, Y can be any value, psi can be any value, x == 0, y == 0: all checked because of above code.
			//dot_X == 0, dot_Y == 0, dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi) == 0, dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			if (Math.abs(this.kineticState.dot_X - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_dot_X = "+this.kineticState.dot_X+" != 0");
			if (Math.abs(this.kineticState.dot_Y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_dot_Y = "+this.kineticState.dot_Y+" != 0");
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_dot_psi = "+this.kineticState.dot_psi+" != 0");
			if (Math.abs(this.kineticState.dot_x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_dot_x = "+this.kineticState.dot_x+" != 0");
			if (Math.abs(this.kineticState.dot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_dot_y = "+this.kineticState.dot_y+" != 0");
			//ddot_X == 0, ddot_Y == 0, ddot_psi == 0, ddot_x == 0, ddot_y == 0
			if (Math.abs(this.kineticState.ddot_X - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_ddot_X = "+this.kineticState.ddot_X+" != 0");
			if (Math.abs(this.kineticState.ddot_Y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_ddot_Y = "+this.kineticState.ddot_Y+" != 0");
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			if (Math.abs(this.kineticState.ddot_x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_ddot_x = "+this.kineticState.ddot_x+" != 0");
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = STOPPED yet new_ddot_y = "+this.kineticState.ddot_y+" != 0");
			break;
			
		case FORWARD_CONSTANT_SPEED:
			//X can be any value, Y can be any value, psi can be any value, x == 0, y == 0: all checked because of above code.
			//dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi) checked because of above code, dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_CONSTANT_SPEED yet new_dot_psi = "+this.kineticState.dot_psi+" != 0");
			if (Math.abs(this.kineticState.dot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_CONSTANT_SPEED yet new_dot_y = "+this.kineticState.dot_y+" != 0");
			//ddot_X == 0, ddot_Y == 0, ddot_psi == 0, ddot_x == 0, ddot_y == 0
			if (Math.abs(this.kineticState.ddot_X - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_CONSTANT_SPEED yet new_ddot_X = "+this.kineticState.ddot_X+" != 0");
			if (Math.abs(this.kineticState.ddot_Y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_CONSTANT_SPEED yet new_ddot_Y = "+this.kineticState.ddot_Y+" != 0");
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_CONSTANT_SPEED yet new_ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			if (Math.abs(this.kineticState.ddot_x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_CONSTANT_SPEED yet new_ddot_x = "+this.kineticState.ddot_x+" != 0");
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_CONSTANT_SPEED yet new_ddot_y = "+this.kineticState.ddot_y+" != 0");
			break;
			
		case FORWARD_ACCELERATE:
			//X can be any value, Y can be any value, psi can be any value, x == 0, y == 0: checked because of above code.
			//dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi) checked because of above code, dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_ACCELERATE yet new_dot_psi = "+this.kineticState.dot_psi+" != 0");
			if (Math.abs(this.kineticState.dot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_ACCELERATE yet new_dot_y = "+this.kineticState.dot_y+" != 0");
			//ddot_psi == 0, ddot_x == get_a(dot_x, Vehicle.this.calculateGearIndexUnderForwardAccelerateDrivingMode(dot_x)), ddot_y == 0
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_ACCELERATE yet new_ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			int gearIndex = Vehicle.this.calculateGearIndexUnderForwardAccelerateDrivingMode(this.kineticState.dot_x);
			tmp = Vehicle.this.get_a(this.kineticState.dot_x, gearIndex);
			if (Math.abs(this.kineticState.ddot_x - tmp) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_ACCELERATE yet ddot_x = "+this.kineticState.ddot_x+" != Vehicle.this.get_a(dot_x = "
						+this.kineticState.dot_x+", gearIndx = "+gearIndex+") = "+tmp);
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_ACCELERATE yet new_ddot_y = "+this.kineticState.ddot_y+" != 0");
			break;
			
		case FORWARD_DECELERATE:
			//X can be any value, Y can be any value, psi can be any value, x == 0, y == 0: checked because of above code.
			//dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi) checked because of above code, dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_DECELERATE yet new_dot_psi = "+this.kineticState.dot_psi+" != 0");
			if (Math.abs(this.kineticState.dot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_DECELERATE yet new_dot_y = "+this.kineticState.dot_y+" != 0");
			//ddot_psi == 0; ddot_x == depends on deceleration stage 1 or 2, and the duration for deceleration, too complicated, give up checking, just require ddot_x <= 0; ddot_y == 0
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_DECELERATE yet new_ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			if (this.kineticState.ddot_x > 0)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_DECELERATE yet ddot_x = "+this.kineticState.ddot_x+" > 0");
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_DECELERATE yet new_ddot_y = "+this.kineticState.ddot_y+" != 0");
			break;
			
		case LANE_CHANGE:
			//X can be any value, Y can be any value, psi can be any value, x == 0, y == 0: checked because of above code.
			//dot_psi too complicated, give up checking; dot_x == dot_X * cos(-psi) - dot_Y sin(-psi) checked because of above code; dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) checked because of above code.
			//ddot_X, ddot_Y, ddot_psi, ddot_x, ddot_y are all too complicated, give up checking.
			break;
			
		case MANUALLY_SET_KINETIC_STATE:
			///do nothing
			//update X;
			this.kineticState.X += this.kineticState.dot_X  * Simulator.getSingletonClock().getTickDuration();
			this.kineticState.Y += this.kineticState.dot_Y  * Simulator.getSingletonClock().getTickDuration();
			//psi remains unchanged;
			break;
			
		default:
			throw new RuntimeException("Unidentified new driving mode.");
		}
		this.lastKineticStateUpdateTimeTick = Simulator.getSingletonClock().getCurrentTick();
	}
	
	/**
	 * Unit: ISO
	 */
	protected synchronized void updateKineticStateWithDrivingModeChange(DrivingMode drivingMode, 
			double X, double Y, double psi, 
			double dot_X, double dot_Y, double dot_psi, 
			double ddot_X, double ddot_Y, double ddot_psi) {
		this.kineticState.drivingMode = drivingMode;
		this.startTimeOfCurrentDrivingModeTick = Simulator.getSingletonClock().getCurrentTick();
		this.updateKineticStateWithoutDrivingModeChange(X, Y, psi, dot_X, dot_Y, dot_psi, ddot_X, ddot_Y, ddot_psi);
	}
	
	/**
	 * Update this.kineticState according to the current 
	 *     this.kineticState.drivingMode, 
	 *     Simulator.getSingletonClock().getCurrentTick(), 
	 *     and this.lastKineticStateupdateTimeTick.
	 * 
	 * @assume Do not call this method when this.kineticState.drivingMode == MANUALLY_SET_KINETIC_STATE.
	 * @assume dot_x does not change during LANE_CHANGE.
	 */
	protected synchronized void updateKineticStateWithoutDrivingModeChange() {
		//sanity check of this.kineticState is supposed to be done during the call to this.setKienticState.
		long curTick = Simulator.getSingletonClock().getCurrentTick();
		long durationTick = curTick - this.lastKineticStateUpdateTimeTick;
		double duration = durationTick * Simulator.getSingletonClock().getTickDuration();
		double tmp = 0;
		switch (this.kineticState.drivingMode) {
		case STOPPED:
			//X remains unchanged, Y remains unchanged, psi remains unchanged.
			//x == 0, y == 0
			if (Math.abs(this.kineticState.x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPEED yet x = "+this.kineticState.x+" != 0");
			else
				this.kineticState.x = 0;
			if (Math.abs(this.kineticState.y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPEED yet y = "+this.kineticState.y+" != 0");
			else
				this.kineticState.y = 0;
			
			//dot_X == 0, dot_Y == 0, dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi) == 0, dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			//As dot_X == dot_Y == 0, there is no need to recalculate dot_x and dot_y: they must == 0.
			if (Math.abs(this.kineticState.dot_X - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet dot_X = "+this.kineticState.dot_X+" != 0");
			else
				this.kineticState.dot_X = 0;
			if (Math.abs(this.kineticState.dot_Y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet dot_Y = "+this.kineticState.dot_Y+" != 0");
			else
				this.kineticState.dot_Y = 0;
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet dot_psi = "+this.kineticState.dot_psi+" != 0");
			else
				this.kineticState.dot_psi = 0;
			if (Math.abs(this.kineticState.dot_x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet dot_x = "+this.kineticState.dot_x+" != 0");
			else
				this.kineticState.dot_x = 0;
			if (Math.abs(this.kineticState.dot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet dot_y = "+this.kineticState.dot_y+" != 0");
			else
				this.kineticState.dot_y = 0;
			
			//ddot_X == 0, ddot_Y == 0, ddot_psi == 0, ddot_x == 0, ddot_y == 0
			//As ddot_X == ddot_Y == 0, there is no need to recalculate ddot_x and ddot_y: they must == 0.
			if (Math.abs(this.kineticState.ddot_X - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet ddot_X = "+this.kineticState.ddot_X+" != 0");
			else
				this.kineticState.ddot_X = 0;
			if (Math.abs(this.kineticState.ddot_Y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet ddot_Y = "+this.kineticState.ddot_Y+" != 0");
			else
				this.kineticState.ddot_Y = 0;
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			else
				this.kineticState.ddot_psi = 0;
			if (Math.abs(this.kineticState.ddot_x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet ddot_x = "+this.kineticState.ddot_x+" != 0");
			else
				this.kineticState.ddot_x = 0;
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = STOPPED yet ddot_y = "+this.kineticState.ddot_y+" != 0");
			else
				this.kineticState.ddot_y = 0;
			break;
			
		case FORWARD_CONSTANT_SPEED:
			this.kineticState.X += this.kineticState.dot_X * duration;
			this.kineticState.Y += this.kineticState.dot_Y * duration;
			//psi remains unchanged.
			//x == 0, y == 0.
			if (Math.abs(this.kineticState.x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet x = "+this.kineticState.x+" != 0");
			else
				this.kineticState.x = 0;
			if (Math.abs(this.kineticState.y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet y = "+this.kineticState.y+" != 0");
			else
				this.kineticState.y = 0;
			
			//dot_X remains unchanged, dot_Y remains unchanged.
			//dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi) remains unchanged, dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet dot_psi = "+this.kineticState.dot_psi+" != 0");
			else
				this.kineticState.dot_psi = 0;
			tmp = this.kineticState.dot_X * Math.cos(-this.kineticState.psi) - this.kineticState.dot_Y * Math.sin(-this.kineticState.psi);
			if (Math.abs(tmp - this.kineticState.dot_x) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet dot_x changed from "+this.kineticState.dot_x+" to "+tmp);
			//else
				//this.kineticState.dot_x = this.kineticState.dot_x, i.e. remains unchanged.
			tmp = this.kineticState.dot_X * Math.sin(-this.kineticState.psi) + this.kineticState.dot_Y * Math.cos(-this.kineticState.psi);
			if (Math.abs(tmp - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet dot_y = "+tmp+" != 0");
			else
				this.kineticState.dot_y = 0;
			
			//ddot_X == 0, ddot_Y == 0, ddot_psi == 0, ddot_x == 0, ddot_y == 0
			//As ddot_X == ddot_Y == 0, there is no need to recalculate ddot_x and ddot_y: they must == 0.
			if (Math.abs(this.kineticState.ddot_X - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet ddot_X = "+this.kineticState.ddot_X+" != 0");
			else
				this.kineticState.ddot_X = 0;
			if (Math.abs(this.kineticState.ddot_Y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet ddot_Y = "+this.kineticState.ddot_Y+" != 0");
			else
				this.kineticState.ddot_Y = 0;
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			else
				this.kineticState.ddot_psi = 0;
			if (Math.abs(this.kineticState.ddot_x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet ddot_x = "+this.kineticState.ddot_x+" != 0");
			else
				this.kineticState.ddot_x = 0;
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_CONSTANT_SPEED yet ddot_y = "+this.kineticState.ddot_y+" != 0");
			else
				this.kineticState.ddot_y = 0;
			break;
			
		case FORWARD_ACCELERATE:
			this.kineticState.X += this.kineticState.dot_X * duration + 0.5 * this.kineticState.ddot_X * duration * duration;
			this.kineticState.Y += this.kineticState.dot_Y * duration + 0.5 * this.kineticState.ddot_Y * duration * duration;
			//psi remains unchanged, 
			//x == 0, y == 0.
			if (Math.abs(this.kineticState.x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_ACCELERATE yet x = "+this.kineticState.x+" != 0");
			else
				this.kineticState.x = 0;
			if (Math.abs(this.kineticState.y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_ACCELERATE yet y = "+this.kineticState.y+" != 0");
			else
				this.kineticState.y = 0;
			
			this.kineticState.dot_X += this.kineticState.ddot_X * duration;
			this.kineticState.dot_Y += this.kineticState.ddot_Y * duration;
			//dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi), dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_ACCELERATE yet new_dot_psi = "+this.kineticState.dot_psi+" != 0");
			else
				this.kineticState.dot_psi = 0;
			this.kineticState.dot_x = this.kineticState.dot_X * Math.cos(-this.kineticState.psi) - this.kineticState.dot_Y * Math.sin(-this.kineticState.psi);
			tmp = this.kineticState.dot_X * Math.sin(-this.kineticState.psi) + this.kineticState.dot_Y * Math.cos(-this.kineticState.psi);
			if (Math.abs(tmp - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_ACCELERATE yet dot_y = "+tmp+" != 0");
			else
				this.kineticState.dot_y = 0;
			
			//ddot_psi == 0, ddot_x == get_a(dot_x, Vehicle.this.calculateGearIndexUnderForwardAccelerateDrivingMode(dot_x)), ddot_y == 0
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_ACCELERATE yet ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			else
				this.kineticState.ddot_psi = 0;
			int gearIndex = Vehicle.this.calculateGearIndexUnderForwardAccelerateDrivingMode(this.kineticState.dot_x);
			this.kineticState.ddot_x = this.get_a(this.kineticState.dot_x, gearIndex);
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" newDrivingMode = FORWARD_ACCELERATE yet ddot_y = "+this.kineticState.ddot_y+" != 0");
			else
				this.kineticState.ddot_y = 0;
			//ddot_X == ddot_x * cos(psi) - ddot_y * sin(psi), ddot_Y == ddot_x * sin(psi) + ddot_y * cos(psi)
			this.kineticState.ddot_X = this.kineticState.ddot_x * Math.cos(this.kineticState.psi) - this.kineticState.ddot_y * Math.sin(this.kineticState.psi);
			this.kineticState.ddot_Y = this.kineticState.ddot_x * Math.sin(this.kineticState.psi) + this.kineticState.ddot_y * Math.cos(this.kineticState.psi); 
			break;
		
		case FORWARD_DECELERATE:
			this.kineticState.X += this.kineticState.dot_X * duration;
			this.kineticState.Y += this.kineticState.dot_Y * duration;
			//psi remains unchanged, 
			//x == 0, y == 0.
			if (Math.abs(this.kineticState.x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_DECELERATE yet x = "+this.kineticState.x+" != 0");
			else
				this.kineticState.x = 0;
			if (Math.abs(this.kineticState.y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_DECELERATE yet y = "+this.kineticState.y+" != 0");
			else
				this.kineticState.y = 0;
			
			this.kineticState.dot_X += this.kineticState.ddot_X * duration;
			this.kineticState.dot_Y += this.kineticState.ddot_Y * duration;
			//dot_psi == 0, dot_x == dot_X * cos(-psi) - dot_Y sin(-psi), dot_y == dot_X sin(-psi) + dot_Y * cos(-psi) == 0
			if (Math.abs(this.kineticState.dot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_DECELERATE yet dot_psi = "+this.kineticState.dot_psi+" != 0");
			else
				this.kineticState.dot_psi = 0;
			this.kineticState.dot_x = this.kineticState.dot_X * Math.cos(-this.kineticState.psi) - this.kineticState.dot_Y * Math.sin(-this.kineticState.psi);
			tmp = this.kineticState.dot_X * Math.sin(-this.kineticState.psi) + this.kineticState.dot_Y * Math.cos(-this.kineticState.psi);
			if (Math.abs(tmp - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_DECELERATE yet dot_y = "+tmp+" != 0");
			else
				this.kineticState.dot_y = 0;
			
			//ddot_psi == 0; ddot_x == depends on deceleration stage 1 or 2, and the duration for deceleration, too complicated, give up checking, just require ddot_x <= 0; ddot_y == 0
			if (Math.abs(this.kineticState.ddot_psi - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_DECELERATE yet ddot_psi = "+this.kineticState.ddot_psi+" != 0");
			else
				this.kineticState.ddot_psi = 0;
			if (this.kineticState.dot_x <= 0)
				this.kineticState.ddot_x = 0;
			else {
				long accumulatedDecelerationDurationTick = curTick - this.startTimeOfCurrentDrivingModeTick;
				if (accumulatedDecelerationDurationTick < 0)
					throw new RuntimeException(this.toString()+" drivingMode = FORWARD_DECELERATE yet accumulated decelertion duration = "+accumulatedDecelerationDurationTick+" < 0 (tick)");
				else if (accumulatedDecelerationDurationTick < this.decelerationStage1DurationTick) 
					this.kineticState.ddot_x = - ((double)accumulatedDecelerationDurationTick) * this.maxDecelerationMagnitude / ((double)this.decelerationStage1DurationTick);
				else //if (accumulatedDecelerationDurationTick >= this.decelerationStage1DurationTick)
					this.kineticState.ddot_x = - this.maxDecelerationMagnitude;
			}
			if (Math.abs(this.kineticState.ddot_y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = FORWARD_DECELERATE yet ddot_y = "+this.kineticState.ddot_y+" != 0");
			else
				this.kineticState.ddot_y = 0;
			this.kineticState.ddot_X = this.kineticState.ddot_x * Math.cos(this.kineticState.psi) - this.kineticState.ddot_y * Math.sin(this.kineticState.psi);
			this.kineticState.ddot_Y = this.kineticState.ddot_x * Math.sin(this.kineticState.psi) + this.kineticState.ddot_y * Math.cos(this.kineticState.psi); 
			break;
			
		case LANE_CHANGE:
			//@see Vehicle Dynamics and Control (2ed) pp30 Eq(2.31). Note $\dot{X} = V_x \cos \psi - V_y \sin \psi; \dot{Y} = V_x \sin \psi + V_y \cos \psi$; V_x is dot_x; V_y is dot_y.
			//Also note pp34, just above Eq(2.38), talks about $V_x$ (i.e. dot_x) can be considered as constant, and $R$ is given by the road shape.
			double k = 0;
			//@assume dot_x does not change during the entire LANE_CHANGE process.
			if (Math.abs(this.kineticState.dot_x - this.get_v_lim()) <= epsilon)
				k = this.k_v_lim;
			else if (Math.abs(this.kineticState.dot_x - this.get_v_low()) <= epsilon)
				k = this.k_v_low;
			else
				throw new RuntimeException(this.toString()+" drivingMode = LANE_CHANGE while dot_x = "+this.kineticState.dot_x+" != v_lim = "+this.get_v_lim()+" nor v_low = "+this.get_v_low());
			System.out.println();
			double delta = this.get_delta(k, this.quarterPeriodTick, Simulator.getSingletonClock().getTickDuration(), this.startTimeOfCurrentDrivingModeTick, curTick);
			System.out.println("RLaneChangingCoefficients at "+Simulator.getSingletonClock().getCurrentTick()+" : "+" k = "+k+" quarterPeriodTick = "+this.quarterPeriodTick
					+" tickDuration = "+Simulator.getSingletonClock().getTickDuration()+" startTimeOfCurrentDrivingModeTick = "+this.startTimeOfCurrentDrivingModeTick+" curTick = "+curTick
					+" delta = "+delta);
			KineticState old = (KineticState) this.kineticState.clone();
			//System.out.print(" get_A_2_2(old.dot_x = "+old.dot_x+" ) = "+this.get_A_2_2(old.dot_x));
			//System.out.print(" get_A_2_4(...) = "+this.get_A_2_4(old.dot_x));
			//System.out.print(" get_B_2_1() = "+this.get_B_2_1());
			this.kineticState.ddot_y = this.get_A_2_2(old.dot_x) * old.dot_y + this.get_A_2_4(old.dot_x) * old.dot_psi + this.get_B_2_1() * delta;
			//System.out.print(" get_A_4_2(old.dot_x = "+old.dot_x+" ) = "+this.get_A_4_2(old.dot_x));
			//System.out.print(" get_A_4_4(...) = "+this.get_A_4_4(old.dot_x));
			//System.out.println(" get_B_4_1() = "+this.get_B_4_1());
			this.kineticState.ddot_psi = this.get_A_4_2(old.dot_x) * old.dot_y + this.get_A_4_4(old.dot_x) * old.dot_psi + this.get_B_4_1() * delta;
			//Also note pp34, just above Eq(2.38), talks about $V_x$ (i.e. dot_x) can be considered as constant, and $R$ is given by the road shape. That is
			this.kineticState.ddot_x = 0;
			//hence we can calculate the following:
			this.kineticState.ddot_X = old.ddot_x * Math.cos(old.psi) - old.dot_x * old.dot_psi * Math.sin(old.psi) - old.ddot_y * Math.sin(old.psi) - old.dot_y * old.dot_psi * Math.cos(old.psi);
			this.kineticState.ddot_Y = old.ddot_x * Math.sin(old.psi) + old.dot_x * old.dot_psi * Math.cos(old.psi) + old.ddot_y * Math.cos(old.psi) - old.dot_y * old.dot_psi * Math.sin(old.psi);
			
			this.kineticState.dot_X += 0.5 * (this.kineticState.ddot_X + old.ddot_X) * duration;
			this.kineticState.dot_Y += 0.5 * (this.kineticState.ddot_Y + old.ddot_Y) * duration;
			this.kineticState.dot_psi += 0.5 * (this.kineticState.ddot_psi + old.ddot_psi) * duration;
			//@see Vehicle Dynamics and Control (2ed) pp34, just above Eq(2.38), talks about regarding dot_x (i.e. $V_x$) as constant
			//so dot_x remains unchanged.
			this.kineticState.dot_y += 0.5 * (this.kineticState.ddot_y + old.ddot_y) * duration;
			
			//x == 0, y == 0: checked because of above code.
			this.kineticState.X += 0.5 * (this.kineticState.dot_X + old.dot_X) * duration;
			this.kineticState.Y += 0.5 * (this.kineticState.dot_Y + old.dot_Y) * duration;
			this.kineticState.psi += 0.5 * (this.kineticState.dot_psi + old.dot_psi) * duration;
			//x == 0, y == 0.
			if (Math.abs(this.kineticState.x - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = LANE_CHANGE yet x = "+this.kineticState.x+" != 0");
			else
				this.kineticState.x = 0;
			if (Math.abs(this.kineticState.y - 0) > epsilon)
				throw new RuntimeException(this.toString()+" drivingMode = LANE_CHANGE yet y = "+this.kineticState.y+" != 0");
			else
				this.kineticState.y = 0;
			break;
			
		case MANUALLY_SET_KINETIC_STATE:
			throw new RuntimeException(this.toString()
					+" drivingMode = MANUALLY_SET_KINETIC_STATE yet you are calling updateKineticStateWithoutDrivingModeChange(), "
					+"you should call the updateKineticStateWithoutDrivingModeChange(...9 parameters...) version");
		
		default: 
			throw new RuntimeException(this.toString()+": unrecognized driving mode = "+this.kineticState.drivingMode);
		}
		this.lastKineticStateUpdateTimeTick = curTick;
	}
	
	/** Duration of stage 1 of deceleration, unit: tick */
	private long decelerationStage1DurationTick = 1;
	public long getDecelerationStage1DurationTick() { return this.decelerationStage1DurationTick; }
	/** The max magnitude of deceleration, unit: m/s^2 */
	private double maxDecelerationMagnitude;
	public double getMaxDecelerationMagnitude() { return this.maxDecelerationMagnitude; }
	
	/** Duration needed for lane change at v_lim, i.e. $delta_{lc}(v_{lim})$, unit: tick */
	private long delta_lc_v_lim_Tick = 1;
	public long get_delta_lc_v_lim_Tick() { return this.delta_lc_v_lim_Tick; }
	/** Duration needed for lane change at v_low, i.e. $delta_{lc}(v_{low})$, unit: tick */
	private long delta_lc_v_low_Tick = 1;
	public long get_delta_lc_v_low_Tick() { return this.delta_lc_v_low_Tick; }
	
	/**
	 * @param delta_lc_v_lim_Tick for $\delta_{lc}(v_{lim})$, i.e. lane change time cost if driving at $v_{lim}$, got via empirical tests or numerical simulations, unit: tick
	 * @param delta_lc_v_low_Tick for $\delta_{lc}(v_{low})$, i.e. lane change time cost if driving at $v_{low}$, got via empirical tests or numerical simulations, unit: tick
	 */
	public Vehicle(double per, 
			double v_lim, double v_low,
			DrivingMode drivingMode, 
			double X, double Y, double psi,
			double dot_X, double dot_Y, double dot_psi,
			double ddot_X, double ddot_Y, double ddot_psi,
			long decelerationStage1DurationTick, double maxDecelerationMagnitude,
			long delta_lc_v_lim_Tick, long delta_lc_v_low_Tick) {
		
		super(per);
		
		u_min = new double[i_g.length];
		u_max = new double[i_g.length];
		lambda = new double[i_g.length];
		for (int i = 0; i < i_g.length; i++) {
			u_min[i] = 0.377 * r * n_min / (i_g[i] * i_0); //see Che Liang Gong Cheng Fang Zhen Yu Fen Xi pp 7 line 1 and 2
			u_max[i] = 0.377 * r * n_max / (i_g[i] * i_0);
			System.out.println(this.toString()+": [u_min, u_max] for gear index "+i+" is ["+u_min[i]+", "+u_max[i]+"](kmh)");
			lambda[i] = 1.03 + 0.04 * i_g[i] * i_g[i]; //see Che Liang Gong Cheng Fang Zhen Yu Fen Xi pp 6.
		}
		
		if (v_lim <= Entity.epsilon || v_low <= Entity.epsilon || v_lim <= v_low+Entity.epsilon) 
			throw new RuntimeException("v_lim = "+v_lim+", v_low = "+v_low);
		this.v_lim = v_lim;
		this.v_low = v_low;
		
		this.updateKineticStateWithDrivingModeChange(drivingMode, X, Y, psi, dot_X, dot_Y, dot_psi, ddot_X, ddot_Y, ddot_psi);
		
		if (decelerationStage1DurationTick < 0) 
			throw new RuntimeException("decelerationStage1DurationTick = "+decelerationStage1DurationTick);
		this.decelerationStage1DurationTick = decelerationStage1DurationTick;
		if (maxDecelerationMagnitude < 0) 
			throw new RuntimeException("maxDecelerationMagnitude = "+maxDecelerationMagnitude);
		this.maxDecelerationMagnitude = maxDecelerationMagnitude;
		
		if (delta_lc_v_lim_Tick < 0) 
			throw new RuntimeException("delta_lc_v_lim_Tick = "+delta_lc_v_lim_Tick);
		this.delta_lc_v_lim_Tick = delta_lc_v_lim_Tick;
		if (delta_lc_v_low_Tick < 0) 
			throw new RuntimeException("delta_lc_v_low_Tick = "+delta_lc_v_low_Tick);
		this.delta_lc_v_low_Tick = delta_lc_v_low_Tick;
	}
}
