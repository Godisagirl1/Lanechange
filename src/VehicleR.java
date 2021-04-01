import java.util.*;

public class VehicleR extends Vehicle{
	public String toString() { return "[R at "+Simulator.getSingletonClock().getCurrentTick()+" tick]"; }
	
	private VehicleRNodeType node = VehicleRNodeType.INIT;
	public boolean isInit() { return (node == VehicleRNodeType.INIT); }
	public enum VehicleRNodeType{
		INIT, L0, REQUESTING_E3, REQUESTING_E4, DEFERRING_LANE_CHANGE, LANE_CHANGING_VLIM, 
		DEFERRING_DECELERATION, DECELERATING, CONST_LOW_SPEED, LANE_CHANGING_VLOW, 
		ACCELERATING_TARGET_LANE, CONST_SPEED_TARGET_LANE
	}
	public String getNodeString() {
		switch(this.node) {
		case INIT: return "INIT";
		case L0: return "L0";
		case REQUESTING_E3: return "REQUESTING_E3";
		case REQUESTING_E4: return "REQUESTING_E4";
		case DEFERRING_LANE_CHANGE: return "DEFERRING_LANE_CHANGE";
		case LANE_CHANGING_VLIM: return "LANE_CHANGING_VLIM";
		case DEFERRING_DECELERATION: return "DEFERRING_DECELERATION";
		case DECELERATING: return "DECELERATING";
		case CONST_LOW_SPEED: return "CONST_LOW_SPEED";
		case LANE_CHANGING_VLOW: return "LANE_CHANGING_VLOW";
		case ACCELERATING_TARGET_LANE: return "ACCELERATING_TARGET_LANE";
		case CONST_SPEED_TARGET_LANE: return "CONST_SPEED_TARGET_LANE";
		}
		throw new RuntimeException(this.toString()+": this.node is at unknown node place.");
	}
	
	/**
	 * @assume The direction of the lane is along the X-axis.
	 */
	public boolean isStablized() {
		boolean result = false;
		if (this.node == VehicleRNodeType.CONST_SPEED_TARGET_LANE) {
			//any X is fine
			//any Y is fine
			if (Math.abs(this.get_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, while psi = "+this.get_psi());
			
			if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, while dot_X = "+this.get_dot_X());
			if (Math.abs(this.get_dot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, while dot_Y = "+this.get_dot_Y());
			if (Math.abs(this.get_dot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, while dot_psi = "+this.get_dot_psi());			
			
			if (Math.abs(this.get_ddot_X() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, while ddot_X = "+this.get_ddot_X());
			if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, while ddot_Y = "+this.get_ddot_Y());
			if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, while ddot_psi = "+this.get_ddot_psi());

			result = true;
		}
		return result;
	}
	
	private boolean lanechangeSucceeded = false;
	
	/** Waiting for Reset1 or Reset2 to happen. See Theorem 1 Claim 2 (Liveness) */
	private boolean waitingForReset = false;
	public boolean isWaitingForReset() { return this.waitingForReset; }
	private long startOfWaitingForResetTick = -1;
	
	/** $D_1$, unit: ISO */
	private double D_1;
	/** $D_3$, unit: ISO */
	private double D_3;
	/** $D_8$, unit: ISO */
	private double D_8;
	
	/** $\Delta_{nonzeno}$, unit: tick */
	private long Delta_nonzeno_Tick;
	/** $\Delta_{reset}$, unit: tick */
	private long Delta_reset_Tick;
	/** $\Delta^*$, unit: tick */
	private long Delta_star_Tick;
	
	/** $\delta_defer_lc$ got from GLOBAL_SEND_LANE_CHANGE_ACPT event, unit: tick */
	private long delta_defer_lc_Tick;
	/** $\delta_defer_Rdec$ got from GLOBAL_SEND_DECELERATE event, unit: tick */
	private long delta_defer_Rdec_Tick;
	/** $\delta_low_LR$ got from GLOAL_SEND_DECELERATE event, unit: tick */
	private long delta_low_LR_Tick;
	
	/** 
	 * Target lane vehicle right in front of me at the time of R_INTEND_CHANGE_LANE event 
	 * null if no such L. 
	 */
	private TargetlaneVehicle L = null;
	/** 
	 * Target lane vehicle right behind me at the time of R_INTEND_CHANGE_LANE event 
	 * null if no such F.
	 */
	private TargetlaneVehicle F = null;
	/** $d_0(R, L)$, POSITIVE_INFINITY if no L exists. Unit: m*/
	private double d_0_R_L = Double.POSITIVE_INFINITY;
	/** $d_0(R, F)$, POSITIVE_INFINITY if no F exists. Unit: m*/
	private double d_0_R_F = Double.POSITIVE_INFINITY;
	
	/** local clock for various temporary uses, unit: tick */
	private long tauTick; 

	/**
	 * Class for the vehicle R.
	 * 
	 * @assume driving along X-axis
	 * 
	 * @param delta_lc_v_lim_Tick for $\delta_{lc}(v_{lim})$, i.e. lane change time cost if driving at $v_{lim}$, got via empirical tests or numerical simulations, unit: tick
	 * @param delta_lc_v_low_Tick for $\delta_{lc}(v_{low})$, i.e. lane change time cost if driving at $v_{low}$, got via empirical tests or numerical simulations, unit: tick
	 */
	public VehicleR(
			//Parameters used by the parent class: Vehicle.
			double per,	double v_lim, double v_low, double initialPosition, 
			long decelerationStage1DurationTick, double maxDecelerationMagnitude,
			long delta_lc_v_lim_Tick, long delta_lc_v_low_Tick,
			
			//Parameters to configure the VehicleR hybrid automaton
			double D_1, double D_3, double D_8,
			long Delta_nonzeno_Tick, long Delta_reset_Tick, long Delta_star_Tick) {
		
		super(per, v_lim, v_low, 
				DrivingMode.FORWARD_CONSTANT_SPEED, initialPosition, 0, 0, v_lim, 0, 0, 0, 0, 0,  
				decelerationStage1DurationTick, maxDecelerationMagnitude, 
				delta_lc_v_lim_Tick, delta_lc_v_low_Tick);

		if (D_1 < 0) throw new RuntimeException(this.toString()+": D_1 = "+D_1);
		this.D_1 = D_1;
		if (D_3 < 0) throw new RuntimeException(this.toString()+": D_3 = "+D_3);
		this.D_3 = D_3;
		if (D_8 < 0) throw new RuntimeException(this.toString()+": D_8 = "+D_8);
		this.D_8 = D_8;
		if (Delta_nonzeno_Tick <= 0) throw new RuntimeException(this.toString()+": Delta_nonzeno_Tick = "+Delta_nonzeno_Tick);
		this.Delta_nonzeno_Tick = Delta_nonzeno_Tick;
		if (Delta_reset_Tick < 0) throw new RuntimeException(this.toString()+": Delta_reset_Tick = "+Delta_reset_Tick);
		this.Delta_reset_Tick = Delta_reset_Tick;
		System.out.println(this.toString()+": Delta_reset_Tick = "+this.Delta_reset_Tick);
		if (Delta_star_Tick < 0) throw new RuntimeException(this.toString()+": Delta_star_Tick = "+Delta_star_Tick);
		this.Delta_star_Tick = Delta_star_Tick;

		this.node = VehicleRNodeType.INIT;
		this.tauTick = (long) (this.Delta_reset_Tick); //intend to change lane from start.
		System.out.println(this.toString()+": tauTick initialized to "+this.tauTick);
	}
	
	public void process(Event event) {
		Event.EventType type = event.getType();
		Entity source = event.getSource();
		ArrayList<Entity> destinations = event.getDestinations();
		if (source == null || destinations == null || !destinations.contains(this))
			throw new RuntimeException("source == null || destinations == null || !destinations.contains(this).");
		if (type == Event.EventType.GLOBAL_TICK) {
			processGlobalTick(event);
		}else if (type == Event.EventType.R_INTEND_CHANGE_LANE) {
			processRIntendChangeLane(event);
		}else if (type == Event.EventType.VR_EVENT1) {
			processVrEvent1(event);
		}else if (type == Event.EventType.VR_EVENT2) {
			processVrEvent2(event);
		}else if (type == Event.EventType.VR_EVENT3) {
			processVrEvent3(event);
		}else if (type == Event.EventType.VR_EVENT4) {
			processVrEvent4(event);
		}else if (type == Event.EventType.GLOBAL_SEND_LANE_CHANGE_ACPT) {
			System.out.println(this.toString()+": received "+event.toString());
			processGlobalSendLaneChangeAcpt(event);
		}else if (type == Event.EventType.R_ACTUAL_LANE_CHANGE) {
			System.out.println(this.toString()+": received "+event.toString());
			processRActualLaneChange(event);
		}else if (type == Event.EventType.GLOBAL_SEND_DECELERATE) {
			System.out.println(this.toString()+": received "+event.toString());
			processGlobalSendDecelerate(event);
		}else if (type == Event.EventType.R_ACTUAL_DECELERATE) {
			System.out.println(this.toString()+": received "+event.toString());
			processRActualDecelerate(event);	
		}else if (type == Event.EventType.R_EVENT1_E3) {
			System.out.println(this.toString()+": received "+event.toString());
			processREvent1_E3(event);
		}else if (type == Event.EventType.R_EVENT1_E4) {
			System.out.println(this.toString()+": received "+event.toString());
			processREvent1_E4(event);	
		}else if (type == Event.EventType.R_EVENT2) {
			System.out.println(this.toString()+": received "+event.toString());
			processREvent2(event);
		}else if (type == Event.EventType.R_EVENT3) {
			System.out.println(this.toString()+": received "+event.toString());
			processREvent3(event);
		}else if (type == Event.EventType.R_EVENT4) {
			System.out.println(this.toString()+": received "+event.toString());
			processREvent4(event);
		}else if (type == Event.EventType.R_EVENT5) {
			System.out.println(this.toString()+": received "+event.toString());
			processREvent5(event);
		}else if (type == Event.EventType.R_EVENT6) {
			System.out.println(this.toString()+": received "+event.toString());
			processREvent6(event);
		}else {
			System.out.println(this.toString()+": received "+event.toString());
		}
	}

	private void processREvent6(Event event) {
		if (this.node != VehicleRNodeType.ACCELERATING_TARGET_LANE) {
			throw new RuntimeException(this.toString()+": received R_EVENT6, while node != ACCELERATING_TARGET_LANE.");
		}
		//node transition
		this.node = VehicleRNodeType.CONST_SPEED_TARGET_LANE;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT6, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT6, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT6, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT6, while dot_psi = "+this.get_dot_psi()+" != 0");
		//ddot_X is too complicated, give up checking, just require >= 0.
		if (this.get_ddot_X() < 0)
			throw new RuntimeException(this.toString()+": received R_EVENT6, while ddot_X = "+this.get_ddot_X()+" < 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT6, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT6, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, 
				this.get_X(), 3.75, 0, 
				this.get_v_lim(), 0, 0, 
				0, 0, 0);
		
		if (lanechangeSucceeded) { // this is the second time R_EVENT6 happened. Wrong!
			throw new RuntimeException(this.toString()+": R_EVENT6 happened again!");
		}else {
			lanechangeSucceeded = true;
			System.out.println(this.toString()+": Lanechange succeeded.");
		}
	}
	
	private void processREvent5(Event event) {
		if (this.node != VehicleRNodeType.LANE_CHANGING_VLOW) {
			throw new RuntimeException(this.toString()+": received R_EVENT5, while node != LANE_CHANGING_VLOW.");
		}
		//node transition
		this.node = VehicleRNodeType.ACCELERATING_TARGET_LANE;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT5, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_low()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT5, while dot_X = "+this.get_dot_X()+" != v_low = "+this.get_v_low());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT5, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT5, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT5, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT5, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT5, while ddot_psi = "+this.get_ddot_psi()+" != 0");
		
		int autoDrivingGearIndex = this.calculateGearIndexUnderForwardAccelerateDrivingMode(this.get_v_low());
		double acceleration = this.get_a(this.get_v_low(), autoDrivingGearIndex);
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_ACCELERATE, 
				this.get_X(), 3.75, 0, 
				this.get_v_low(), 0, 0, 
				acceleration, 0, 0);
	}
	
	private void processREvent4(Event event) {
		if (this.node != VehicleRNodeType.CONST_LOW_SPEED) {
			throw new RuntimeException(this.toString()+": received R_EVENT4, while node != CONST_LOW_SPEED.");
		}
		//node transition
		this.node = VehicleRNodeType.LANE_CHANGING_VLOW;
		this.tauTick = 0;

		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT4, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_low()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT4, while dot_X = "+this.get_dot_X()+" != v_low = "+this.get_v_low());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT4, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT4, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT4, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT4, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT4, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.LANE_CHANGE, this.get_X(), this.get_Y(), 0, this.get_v_low(), 0, 0, 0, 0, 0);
	}
	
	private void processREvent3(Event event) {
		if (this.node != VehicleRNodeType.DECELERATING) {
			throw new RuntimeException(this.toString()+": received R_EVENT3, while node != DECELERATING.");
		}
		//node transition
		this.node = VehicleRNodeType.CONST_LOW_SPEED;
		this.tauTick = 0;
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_low(), 0, 0, 
				0, 0, 0);
	}
	
	private void processREvent2(Event event) {
		if (this.node != VehicleRNodeType.LANE_CHANGING_VLIM) {
			throw new RuntimeException(this.toString()+": received R_EVENT2, while node != LANE_CHANGING_VLIM.");
		}
		//node transition
		this.node = VehicleRNodeType.CONST_SPEED_TARGET_LANE;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT2, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT2, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT2, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT2, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT2, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT2, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT2, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, this.get_X(), 3.75, 0, this.get_v_lim(), 0, 0, 0, 0, 0);
		
		if (lanechangeSucceeded) { // this is the second time R_EVENT2 happened. Wrong!
			throw new RuntimeException(this.toString()+": R_EVENT2 happened again!");
		}else {
			lanechangeSucceeded = true;
			System.out.println(this.toString()+": Lanechange succeeded.");
		}
	}
	
	private void processREvent1_E3(Event event) {
		if (this.node != VehicleRNodeType.REQUESTING_E3) {
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while node != REQUESTING_E3.");
		}
		//node transition
		this.node = VehicleRNodeType.INIT;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E3, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, this.get_X(), this.get_Y(), 0, this.get_v_lim(), 0, 0, 0, 0, 0);
	}
	
	private void processREvent1_E4(Event event) {
		if (this.node != VehicleRNodeType.REQUESTING_E4) {
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while node != REQUESTING_E4.");
		}
		//node transition
		this.node = VehicleRNodeType.INIT;
		this.tauTick = 0;		
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, this.get_X(), this.get_Y(), 0, this.get_v_lim(), 0, 0, 0, 0, 0);
	}
	
	private void processRActualDecelerate(Event event) {
		if (this.node != VehicleRNodeType.DEFERRING_DECELERATION) {
			throw new RuntimeException(this.toString()+": received R_ACTUAL_DECELERATE, while node != DEFERRING_DECELERATION.");
		}
		//node transition
		this.node = VehicleRNodeType.DECELERATING;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_EVENT1_E4, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_DECELERATE, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_lim(), 0, 0, 
				0, 0, 0);
	}
	
	/**
	 * @assume driving along X-axis
	 */
	private void processGlobalSendDecelerate(Event event) {
		if (this.node != VehicleRNodeType.REQUESTING_E4) {
			System.out.println(this.toString()+": received GLOBAL_SEND_DECELERATE, while node != REQUESTING_E4, ignore event.");
			return; //ignore the event
		}
		if (Math.random() > this.per) { //The message did get through the wireless medium
			Entity source = event.getSource();
			ArrayList<Entity> destinations = event.getDestinations();
			if (destinations == null || !destinations.contains(this))
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, but the destinations is null or does not contain me.");
			String content = event.getContent();
			StringTokenizer st = new StringTokenizer(content);
			String tmp = st.nextToken();
			if (tmp == null) throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, but no delta_defer_Rdec_Tick info in the content.");
			this.delta_defer_Rdec_Tick = Long.parseLong(tmp);
			tmp = st.nextToken();
			if (tmp == null) throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, but no delta_low_LR_Tick info in the content.");
			this.delta_low_LR_Tick = Long.parseLong(tmp);
			//node transition
			this.node = VehicleRNodeType.DEFERRING_DECELERATION;
			this.tauTick = 0;
			
			//no need to check X, Y
			if (Math.abs(this.get_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, while psi = "+this.get_psi()+" != 0");
			if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
			if (Math.abs(this.get_dot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, while dot_Y = "+this.get_dot_Y()+" != 0");
			if (Math.abs(this.get_dot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, while dot_psi = "+this.get_dot_psi()+" != 0");
			if (Math.abs(this.get_ddot_X() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, while ddot_X = "+this.get_ddot_X()+" != 0");
			if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, while ddot_Y = "+this.get_ddot_Y()+" != 0");
			if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_DECELERATE, while ddot_psi = "+this.get_ddot_psi()+" != 0");
			
			//correct the accumulated double computation errors.
			this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, 
					this.get_X(), this.get_Y(), 0, 
					this.get_v_lim(), 0, 0, 
					0, 0, 0);
		}else {
			System.out.println(this.toString()+": GLOBAL_SEND_DECELERATE happened, but lost due to PER, ignore event.");
		}
	}
	
	private void processRActualLaneChange(Event event) {
		if (this.node != VehicleRNodeType.DEFERRING_LANE_CHANGE) {
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while node != DEFERRING_LANE_CHANGE.");
		}
		//node transition
		this.node = VehicleRNodeType.LANE_CHANGING_VLIM;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received R_ACTUAL_LANE_CHANGE, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.LANE_CHANGE, this.get_X(), this.get_Y(), 0, this.get_v_lim(), 0, 0, 0, 0, 0);
	}
	
	private void processGlobalSendLaneChangeAcpt(Event event) {
		if (this.node != VehicleRNodeType.REQUESTING_E3) {
			System.out.println(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while node != REQUESTING_E3, ignore event.");
			return; //ignore the event
		}
		if (Math.random() > this.per) { //The message did get through the wireless medium
			Entity source = event.getSource();
			ArrayList<Entity> destinations = event.getDestinations();
			if (destinations == null || !destinations.contains(this))
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, but the destinations is null or does not contain me.");
			String content = event.getContent();
			this.delta_defer_lc_Tick = Long.parseLong(content);
			//node transition
			this.node = VehicleRNodeType.DEFERRING_LANE_CHANGE;
			this.tauTick = 0;
			
			//no need to check X, Y
			if (Math.abs(this.get_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while psi = "+this.get_psi()+" != 0");
			if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
			if (Math.abs(this.get_dot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while dot_Y = "+this.get_dot_Y()+" != 0");
			if (Math.abs(this.get_dot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while dot_psi = "+this.get_dot_psi()+" != 0");
			if (Math.abs(this.get_ddot_X() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while ddot_X = "+this.get_ddot_X()+" != 0");
			if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while ddot_Y = "+this.get_ddot_Y()+" != 0");
			if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_ACPT, while ddot_psi = "+this.get_ddot_psi()+" != 0");
			
			//correct the accumulated double computation errors.
			this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, 
					this.get_X(), this.get_Y(), 0, 
					this.get_v_lim(), 0, 0, 
					0, 0, 0);
		}else {
			System.out.println(this.toString()+": GLOBAL_SEND_LANE_CHANGE_ACPT event happened, but message lost due to PER, ignore event.");
		}
	}
	
	/**
	 * @return L, the target vehicle right ahead of this vehicle R. null if such L does not exist.
	 */
	private TargetlaneVehicle findL(){
		TargetlaneVehicle L = null;
		ArrayList<TargetlaneVehicle> targetLaneVehicles = Simulator.getTargetLaneVehicles();
		
		double X_R = this.get_X();
		for (int i = 0; i < targetLaneVehicles.size(); i++) {
			TargetlaneVehicle v = targetLaneVehicles.get(i);
			double X_v = v.get_X();
			if (X_v < X_R) continue; //in case X_v == X_R, v is considered an L, not an F
			if (L == null) {
				L = v;
			} else {
				if (X_v < L.get_X()) {
					L = v;
				}
			}
		}
		return L;
	}
	
	/**
	 * @return F, the target vehicle right behind this vehicle R. null if such F does not exist.
	 */
	private TargetlaneVehicle findF(){
		TargetlaneVehicle F = null;
		ArrayList<TargetlaneVehicle> targetLaneVehicles = Simulator.getTargetLaneVehicles();
		
		double X_R = this.get_X();
		for (int i = 0; i < targetLaneVehicles.size(); i++) {
			TargetlaneVehicle v = targetLaneVehicles.get(i);
			double X_v = v.get_X();
			if (X_v >= X_R) continue; //in case X_v == X_R, v is considered an L, not an F
			if (F == null) {
				F = v;
			} else {
				if (X_v > F.get_X()) {
					F = v;
				}
			}
		}
		return F;
	}
	
	
	private void processVrEvent4(Event event) {
		if (this.node != VehicleRNodeType.L0)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while node != L0.");
		
		//L and F, the leader and follower vehicle on the target lane; already found during the most recent R_INTEND_CHANGE_LANE event.
		///TargetlaneVehicle L = this.findL();
		///TargetlaneVehicle F = this.findF();
		if (this.L == null) throw new RuntimeException(this.toString()+": received VR_EVENT4, but no L found.");
		///double d_0_R_L = L.get_X() - this.get_X();
		if (this.d_0_R_L < 0) throw new RuntimeException(this.toString()+": find an L whose X < R's X.");
		if (this.F == null) throw new RuntimeException(this.toString()+": received VR_EVENT4, but no F found.");
		///double d_0_R_F = this.get_X() - F.get_X();
		if (this.d_0_R_F <= 0) throw new RuntimeException(this.toString()+": find an F whose X >= R's X.");
		
		ArrayList<Entity> destinations = new ArrayList<Entity>();
		destinations.add(this.F);
		Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(),
				Event.EventType.GLOBAL_SEND_LANE_CHANGE_REQ, this, destinations, this.d_0_R_L+" "+this.d_0_R_F);
		Simulator.addEvent(nextEvent);
		System.out.println(this.toString()+": added "+nextEvent.toString());
		//node transition
		this.node = VehicleRNodeType.REQUESTING_E4;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT4, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, this.get_X(), this.get_Y(), 0, this.get_v_lim(), 0, 0, 0, 0, 0);
	}
	
	private void processVrEvent3(Event event) {
		if (this.node != VehicleRNodeType.L0)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while node != L0.");

		//L and F, the leader and follower vehicle on the target lane; already found during the most recent R_INTEND_CHANGE_LANE event.
		///TargetlaneVehicle L = this.findL();
		///TargetlaneVehicle F = this.findF();
		///double d_0_R_L = Double.POSITIVE_INFINITY;
		///if (L != null) d_0_R_L = L.get_X() - this.get_X();
		if (this.L == null && this.d_0_R_L < Double.POSITIVE_INFINITY) throw new RuntimeException(this.toString()+": no L, but d_0_R_L == "+this.d_0_R_L);
		if (this.L != null && this.d_0_R_L < 0) throw new RuntimeException(this.toString()+": find an L whose X < R's X.");
		if (this.F == null) throw new RuntimeException(this.toString()+": received VR_EVENT3, but no F found.");
		///double d_0_R_F = this.get_X() - F.get_X();
		if (this.d_0_R_F <= 0) throw new RuntimeException(this.toString()+": find an F whose X >= R's X.");
		
		ArrayList<Entity> destinations = new ArrayList<Entity>();
		destinations.add(this.F);
		Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(),
				Event.EventType.GLOBAL_SEND_LANE_CHANGE_REQ, this, destinations, this.d_0_R_L+" "+this.d_0_R_F);
		Simulator.addEvent(nextEvent);
		System.out.println(this.toString()+": added "+nextEvent.toString());
		//node transition
		this.node = VehicleRNodeType.REQUESTING_E3;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT3, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_CONSTANT_SPEED, this.get_X(), this.get_Y(), 0, this.get_v_lim(), 0, 0, 0, 0, 0);
	}
	
	private void processVrEvent2(Event event) {
		if (this.node != VehicleRNodeType.L0)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while node != L0.");
		
		//L and F, the leader and follower vehicle on the target lane; already found during the most recent R_INTEND_CHANGE_LANE event.
		///TargetlaneVehicle L = this.findL();
		///TargetlaneVehicle F = this.findF();
		if (this.L == null) throw new RuntimeException(this.toString()+": received VR_EVENT2, but no L found.");
		///double d_0_R_L = L.get_X() - this.get_X();
		if (this.d_0_R_L < 0) throw new RuntimeException(this.toString()+": find an L whose X < R's X.");
		///double d_0_R_F = Double.POSITIVE_INFINITY;
		if (this.F == null && this.d_0_R_F < Double.POSITIVE_INFINITY) throw new RuntimeException(this.toString()+": F does not exist but d_0_R_F == "+this.d_0_R_F);
		///if (F != null) d_0_R_F = this.get_X() - F.get_X();
		if (this.F != null && this.d_0_R_F <= 0) throw new RuntimeException(this.toString()+": find an F whose X >= R's X.");
		
		double tmp = (this.get_v_lim() * this.Delta_star_Tick * Simulator.getSingletonClock().getTickDuration() - this.d_0_R_L) / (this.get_v_lim() - this.get_v_low()); //unit: ISO
		this.delta_low_LR_Tick = (long)Math.round(tmp / Simulator.getSingletonClock().getTickDuration());
						
		//node transition
		this.node = VehicleRNodeType.DECELERATING;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT2, while ddot_psi = "+this.get_ddot_psi()+" != 0");
				
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_DECELERATE, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_lim(), 0, 0, 
				0, 0, 0);
	}
	
	private void processVrEvent1(Event event) {
		if (this.node != VehicleRNodeType.L0)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while node != L0.");
		
		//node transition
		this.node = VehicleRNodeType.LANE_CHANGING_VLIM;
		this.tauTick = 0;
		
		//no need to check X, Y
		if (Math.abs(this.get_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while psi = "+this.get_psi()+" != 0");
		if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while dot_X = "+this.get_dot_X()+" != v_lim = "+this.get_v_lim());
		if (Math.abs(this.get_dot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while dot_Y = "+this.get_dot_Y()+" != 0");
		if (Math.abs(this.get_dot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while dot_psi = "+this.get_dot_psi()+" != 0");
		if (Math.abs(this.get_ddot_X() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while ddot_X = "+this.get_ddot_X()+" != 0");
		if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while ddot_Y = "+this.get_ddot_Y()+" != 0");
		if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
			throw new RuntimeException(this.toString()+": received VR_EVENT1, while ddot_psi = "+this.get_ddot_psi()+" != 0");
		
		this.updateKineticStateWithDrivingModeChange(DrivingMode.LANE_CHANGE, this.get_X(), this.get_Y(), 0, this.get_v_lim(), 0, 0, 0, 0, 0);
	}
			
	private void processRIntendChangeLane(Event event) {
		if (this.node != VehicleRNodeType.INIT)
			throw new RuntimeException(this.toString()+": received R_INTEND_CHANGE_LANE, while node != INIT.");

		if (Simulator.allTargetlaneVehiclesStablized()) {
				System.out.println(this.toString()+": starts waiting for reset.");
				this.waitingForReset = true;
				this.startOfWaitingForResetTick = Simulator.getSingletonClock().getCurrentTick();
		}
		
		//Find L and F, the leader and follower vehicle on the target lane; null if not found.
		this.L = this.findL();
		this.F = this.findF();
		if (this.L == null) this.d_0_R_L = Double.POSITIVE_INFINITY;
		else this.d_0_R_L = this.L.get_X() - this.get_X();
		if (this.d_0_R_L < 0) throw new RuntimeException(this.toString()+": find an L whose X < R's X.");
		if (this.F == null) this.d_0_R_F = Double.POSITIVE_INFINITY;
		else this.d_0_R_F = this.get_X() - this.F.get_X();
		if (this.d_0_R_F <= 0) throw new RuntimeException(this.toString()+": find an F whose X >= R's X.");
				
		// node transition
		this.node = VehicleRNodeType.L0;
		//note node L0 is a transient node, the entrance to L0 immediately triggers the next event.
		ArrayList<Entity> nextEventDestination = new ArrayList<Entity>();
		nextEventDestination.add(this);
		Event nextEvent = null;
		if (this.d_0_R_L >= this.D_1 && this.d_0_R_F >= this.D_3) {
			nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.VR_EVENT1, this, nextEventDestination, "");
		} else if (this.d_0_R_L < this.D_1 && this.d_0_R_F >= this.D_8) {
			nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.VR_EVENT2, this, nextEventDestination, "");
		} else if (this.d_0_R_L >= this.D_1 && this.d_0_R_F < this.D_3) {
			nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.VR_EVENT3, this, nextEventDestination, "");	
		} else {
			nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.VR_EVENT4, this, nextEventDestination, "");
		}
		Simulator.addEvent(nextEvent);
		System.out.println(this.toString()+": added "+nextEvent.toString());
	}
	
	/**
	 * @assume driving along the X-axis
	 */
	private void processGlobalTick(Event event) {
		if (Simulator.timeToLog())	System.out.println(this.toStringLong());
		if (this.node == VehicleRNodeType.INIT) {
			this.tauTick++;			
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.tauTick >= this.Delta_reset_Tick) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_INTEND_CHANGE_LANE, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}

			if (this.waitingForReset) {
				if (Simulator.getVehicleR().isInit() && Simulator.allTargetlaneVehiclesStablized()) {
					System.out.print(this.toString()+": Reset 1 happened in ");
					long resetTimeCostTick = Simulator.getSingletonClock().getCurrentTick() - this.startOfWaitingForResetTick;
					System.out.println(resetTimeCostTick+" ticks.");
					this.waitingForReset = false;
					this.startOfWaitingForResetTick = -1;
				}else if (Simulator.allVehiclesStablized()) {
					System.out.print(this.toString()+": Reset 2 happened in ");
					long resetTimeCostTick = Simulator.getSingletonClock().getCurrentTick() - this.startOfWaitingForResetTick;
					System.out.println(resetTimeCostTick+" ticks.");
					this.waitingForReset = false;
					this.startOfWaitingForResetTick = -1;
				}//else do nothing.
			}

		}else if (this.node == VehicleRNodeType.L0) {
			//can receive at the most one, but should not receive two or more clock ticks when in L0.
			System.out.println(this.toString()+": received GLOBAL_TICK while node = L0.");
		}else if (this.node == VehicleRNodeType.REQUESTING_E3) {
			this.tauTick++; //all events (particularly the node transition events) other than 
			//the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.tauTick >= this.Delta_nonzeno_Tick) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_EVENT1_E3, this, destinations, "");
				Simulator.addEvent(nextEvent);
				//System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.REQUESTING_E4) {
			this.tauTick++; //all events (particularly the node transition events) other than 
			//the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.tauTick >= this.Delta_nonzeno_Tick) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_EVENT1_E4, this, destinations, "");
				Simulator.addEvent(nextEvent);
				//System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.DEFERRING_LANE_CHANGE) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.tauTick >= this.delta_defer_lc_Tick) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_ACTUAL_LANE_CHANGE, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.LANE_CHANGING_VLIM) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.tauTick >= this.get_delta_lc_v_lim_Tick()) {
				//correct the accumulated double computation errors. 
				//Note we do not correct the accumulated double computation errors on Y. This may be a bug in the future.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						0, 0, 0);
				System.out.println(this.toString()+": finished lane change in v_lim, dot_X = "+this.get_dot_X()+" dot_Y = "+this.get_dot_Y()+" dot_psi = "+this.get_dot_psi());
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_EVENT2, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.DEFERRING_DECELERATION) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.tauTick >= this.delta_defer_Rdec_Tick) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_ACTUAL_DECELERATE, this, destinations);
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.DECELERATING) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.getSpeed() <= this.get_v_low() ) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_low(), 0, 0, 
						this.get_ddot_X(), 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_EVENT3, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.CONST_LOW_SPEED) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (tauTick >= this.delta_low_LR_Tick) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_low(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_EVENT4, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.LANE_CHANGING_VLOW) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.tauTick >= this.get_delta_lc_v_low_Tick()) { 
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_low(), 0, 0, 
						0, 0, 0);
				System.out.println(this.toString()+": finished lane change in vLow, dot_X = "+this.get_dot_X()+" dot_Y = "+this.get_dot_Y()+" dot_psi = "+this.get_dot_psi());
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_EVENT5, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}	
		}else if (this.node == VehicleRNodeType.ACCELERATING_TARGET_LANE) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			if (this.getSpeed() >= this.get_v_lim()) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						this.get_ddot_X(), 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.R_EVENT6, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == VehicleRNodeType.CONST_SPEED_TARGET_LANE) {
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happen right after a triggering GLOBAL_TICK event.	
			this.updateKineticStateWithoutDrivingModeChange();
			if (Math.abs(this.getSpeed() - this.get_v_lim()) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, dot_X = "+this.get_dot_X()+" dot_Y = "+this.get_dot_Y()+" dot_psi = "+this.get_dot_psi());
			if (Math.abs(this.get_ddot_X() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, ddot_X = "+this.get_ddot_X());
			if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, ddot_Y = "+this.get_ddot_Y());
			if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_SPEED_TARGET_LANE, ddot_psi = "+this.get_ddot_psi());

			if (this.waitingForReset) {
				if (Simulator.allVehiclesStablized()){
					System.out.print(this.toString()+": Reset 2 happened in ");
					long resetTimeCostTick = Simulator.getSingletonClock().getCurrentTick() - this.startOfWaitingForResetTick;
					System.out.println(resetTimeCostTick+" ticks.");
					this.waitingForReset = false;
					this.startOfWaitingForResetTick = -1;
				}
			}	
		}
	}
}

