import java.util.*;

public class TargetlaneVehicle extends Vehicle{
	private int id = -1;
	public int getId() { return this.id; }
	public String toString() { 
		return "[Tar "+this.id+" at "+Simulator.getSingletonClock().getCurrentTick()+" tick]"; 
	}

	private TargetlaneVehicle predecessor;
	public void setPredecessor(TargetlaneVehicle predecessor) {
		if (predecessor != null && predecessor.getId()+1 != this.getId())
			throw new RuntimeException("predecessor != null && predecessor.getId() = "
					+predecessor.getId()+", while this.getId() = "+this.getId());
		this.predecessor = predecessor;
		checkPredecessorValidity();
	}
	/**
	 * Assumes target lane vehicles are always driving forward on X-axis.
	 * @throws RuntimeException if predecessor position is invalid.
	 */
	public void checkPredecessorValidity() {
		if (predecessor != null && this.get_X() >= predecessor.get_X())
			throw new RuntimeException(this.toString()+": predecessor != null && predecessor.get_X() = "
					+predecessor.get_X()+", while this.get_X() = "+this.get_X());
	}
	private TargetlaneVehicle successor;
	public void setSuccessor(TargetlaneVehicle successor) {
		if (successor != null && successor.getId()-1 != this.getId())
			throw new RuntimeException("successor != null && successor.getId() = "
					+successor.getId()+", while this.getId() = "+this.getId());
		this.successor = successor;
		checkSuccessorValidity();
	}
	/**
	 * Assumes target lane vehicles are always driving forward on X-axis.
	 * @throws RuntimeException if successor position is invalid.
	 */
	public void checkSuccessorValidity() {
		if (successor != null && this.get_X() <= successor.get_X())
			throw new RuntimeException("successor != null && successor.getPosition() = "
					+successor.get_X()+", while this.getPosition() = "+this.get_X());
	}
	
	private TargetlaneVehicleNodeType node = TargetlaneVehicleNodeType.INIT;
	
	public enum TargetlaneVehicleNodeType{
		INIT, VALIDATION, DEFERRING_DECELERATION, DECELERATING_1, DECELERATING_2, 
		CONST_LOW_SPEED_1, CONST_LOW_SPEED_2, ACCELERATING, SYNC
	}
	
	public String getNodeString() {
		switch (this.node) {
		case INIT: return "INIT";
		case VALIDATION: return "VALIDATION";
		case DECELERATING_1: return "DECELERATING_1";
		case DECELERATING_2: return "DECELERATING_2";
		case CONST_LOW_SPEED_1: return "CONST_LOW_SPEED_1";
		case CONST_LOW_SPEED_2: return "CONST_LOW_SPEED_2";
		case ACCELERATING: return "ACCELERATING";
		case SYNC: return "SYNC";
		}
		throw new RuntimeException(this.toString()+": this.node is at unknown node place.");
	}
	
	/**
	 * @assume moving along X-axis when stabilized.
	 */
	public boolean isStablized() {
		boolean result = false;
		if (this.node == TargetlaneVehicleNodeType.INIT) {
			//any X is fine
			//any Y is fine
			if (Math.abs(this.get_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, while psi = "+this.get_psi());
			
			if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, while dot_X = "+this.get_dot_X());
			if (Math.abs(this.get_dot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, while dot_Y = "+this.get_dot_Y());
			if (Math.abs(this.get_dot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, while dot_psi = "+this.get_dot_psi());			
			
			if (Math.abs(this.get_ddot_X() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, while ddot_X = "+this.get_ddot_X());
			if (Math.abs(this.get_ddot_Y() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, while ddot_Y = "+this.get_ddot_Y());
			if (Math.abs(this.get_ddot_psi() - 0) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, while ddot_psi = "+this.get_ddot_psi());

			result = true;
		}
		return result;
	}
	
	/** $D_1, unit: m */
	private double D_1 = 0;
	/** $D_3, unit: m */
	private double D_3 = 0;
	/** $D_8, unit: m */
	private double D_8 = 0;
	/** $D_Sync$, unit: m */
	private double D_Sync = 0;
	/** $\Delta^*$, unit: tick */
	private long Delta_star_Tick = 0;
	/** Reference pointing the vehicle R. */
	private VehicleR R = null;
	/** $d_0(R, L)$ got from GLOBAL_SEND_LANE_CHANGE_REQ, unit: m */
	private double d_0_R_L = 0;
	/** $d_0(R, F)$ got from GLOBAL_SEND_LANE_CHANGE_REQ, unit: m */
	private double d_0_R_F = 0;
	/** $d_d(v_{lim}, v_{low})$, unit: m */
	private double d_d_v_lim_v_low = 0;
	/** $\delta_d(v_{lim}, v_{low})$, unit: Tick */
	private long delta_d_v_lim_v_low_Tick = 0;
	/** $\delta_defer_Rdec$, unit: tick */
	private long delta_defer_Rdec_Tick = 0;
	/**  $\delta_defer_lc$, unit: tick */
	private long delta_defer_lc_Tick = 0;
	/** $\delta_low_LR$, unit: tick */
	private long delta_low_LR_Tick = 0;
	/** $\delta_low_RF1$, unit: tick */
	private long delta_low_RF1_Tick = 0;
	/** $\delta_low_RF2$, unit: tick */
	private long delta_low_RF2_Tick = 0;
	/** a timer for temporary usage here and there. Unit: tick */
	private long tauTick = 0;
	
	private String state = "Init";
		
	/**
	 * Class for targetlane vehicles.
	 * 
	 * @assume driving along X-axis
	 * 
	 * @param delta_lc_v_lim_Tick for $\delta_{lc}(v_{lim})$, i.e. lane change time cost if driving at $v_{lim}$, got via empirical tests or numerical simulations, unit: tick
	 * @param delta_lc_v_low_Tick for $\delta_{lc}(v_{low})$, i.e. lane change time cost if driving at $v_{low}$, got via empirical tests or numerical simulations, unit: tick
	 */
	public TargetlaneVehicle(
			//Parameters used by the parent class: Vehicle.
			double per, double v_lim, double v_low, double initialPosition, 
			long decelerationStage1DurationTick, double maxDecelerationMagnitude,
			long delta_lc_v_lim_Tick, long delta_lc_v_low_Tick,
			
			//Parameters to configure the TargetlaneVehicle hybrid automaton
			int id, 
			double D_1, double D_3, double D_8,
			double D_Sync, long Delta_star_Tick,
			double d_d_v_lim_v_low, long delta_d_v_lim_v_low_Tick) {
		
		super(per, v_lim, v_low, Vehicle.DrivingMode.FORWARD_CONSTANT_SPEED, 
				initialPosition, 3.75, 0, v_lim, 0, 0, 0, 0, 0,
				decelerationStage1DurationTick, maxDecelerationMagnitude,
				delta_lc_v_lim_Tick, delta_lc_v_low_Tick);
		
		if (id < 0) throw new RuntimeException("Targetlane Vehicle ID: "+id+" < 0.");
		this.id = id;
		
		if (D_1 <= 0) throw new RuntimeException("D_1 = "+D_1);
		this.D_1 = D_1;
		if (D_3 <= 0) throw new RuntimeException("D_3 = "+D_3);
		this.D_3 = D_3;
		if (D_8 <= 0) throw new RuntimeException("D_8 = "+D_8);
		this.D_8 = D_8;
		if (D_Sync <= 0) throw new RuntimeException("D_Sync = "+D_Sync);
		this.D_Sync = D_Sync;
		if (Delta_star_Tick <= 0) throw new RuntimeException("Delta_star_Tick = "+Delta_star_Tick);
		this.Delta_star_Tick = Delta_star_Tick;
		if (d_d_v_lim_v_low <= 0) throw new RuntimeException("d_d_v_lim_v_low = "+d_d_v_lim_v_low);
		this.d_d_v_lim_v_low = d_d_v_lim_v_low;
		if (delta_d_v_lim_v_low_Tick <= 0) throw new RuntimeException("delta_d_v_lim_v_low_Tick = "+delta_d_v_lim_v_low_Tick);
		this.delta_d_v_lim_v_low_Tick = delta_d_v_lim_v_low_Tick;
		
		this.node = TargetlaneVehicleNodeType.INIT;
		this.state = "Init";
		this.tauTick = 0;
	}
	
	public void process(Event event) {
		Event.EventType type = event.getType();
		Entity source = event.getSource();
		ArrayList<Entity> destinations = event.getDestinations();
		if (source == null || destinations == null || !destinations.contains(this))
			throw new RuntimeException("source == null || destinations == null || !destinations.contains(this).");
		if (type == Event.EventType.GLOBAL_TICK) {
			processGlobalTick(event);
		}else if (type == Event.EventType.GLOBAL_SEND_LANE_CHANGE_REQ) {
			System.out.println(this.toString()+": received "+event.toString());
			processGlobalSendLaneChangeReq(event);
		}else if (type == Event.EventType.TAR_EVENT1) {
			System.out.println(this.toString()+": received "+event.toString());
			processTarEvent1(event);
		}else if (type == Event.EventType.TAR_EVENT2) {
			System.out.println(this.toString()+": received "+event.toString());
			processTarEvent2(event);
		}else if (type == Event.EventType.TAR_VALIDATION_FAILURE) {
			System.out.println(this.toString()+": received "+event.toString());
			processTarValidationFailure(event);
		}else if (type == Event.EventType.TAR_START_SYNC_PRED) {
			System.out.println(this.toString()+": received "+event.toString());
			processTarStartSyncPred(event);
		}else if (type == Event.EventType.TAR_STOP_SYNC_PRED) {
			System.out.println(this.toString()+": received "+event.toString());
			processTarStopSyncPred(event);
		}else if (type == Event.EventType.T_EVENT2) {
			System.out.println(this.toString()+": received "+event.toString());
			processTEvent2(event);
		}else if (type == Event.EventType.T_EVENT3) {
			System.out.println(this.toString()+": received "+event.toString());
			processTEvent3(event);
		}else if (type == Event.EventType.T_EVENT4) {
			System.out.println(this.toString()+": received "+event.toString());
			processTEvent4(event);	
		}else if (type == Event.EventType.T_EVENT5) {
			System.out.println(this.toString()+": received "+event.toString());
			processTEvent5(event);	
		}else if (type == Event.EventType.T_EVENT6) {
			System.out.println(this.toString()+": received "+event.toString());
			processTEvent6(event);
		}else {
			throw new RuntimeException(this.toString()+": unrecognized event "+event.toString());
		}
	}
	
	/**
	 * @assume driving along X-axis.
	 */
	private void processTarStartSyncPred(Event event) {
		if (this.node != TargetlaneVehicleNodeType.INIT) {
			System.out.println(this.toString()+": received TAR_START_SYNC_PRED, while node != INIT, ignore event.");
			return; //ignore the event;
		}
		if (this.predecessor == null) 
			throw new RuntimeException(this.toString()+": received TAR_START_SYNC_PRED, while precedessor = null.");
		double distanceToPredecessor = this.predecessor.get_X() - this.get_X();
		if (distanceToPredecessor <= 0) 
			throw new RuntimeException(this.toString()+": received TAR_START_SYNC_PRED, while distance to predecessor = "+distanceToPredecessor);
		if (distanceToPredecessor < this.D_Sync) {
			this.node = TargetlaneVehicleNodeType.SYNC;
			this.state = "Sync";
			//correct the accumulated double computation errors.
			this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.MANUALLY_SET_KINETIC_STATE, 
					this.get_X(), this.get_Y(), 0, 
					this.get_v_lim(), 0, 0, 
					0, 0, 0);
			if (this.successor != null) {
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this.successor);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.TAR_START_SYNC_PRED, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		} else {
			System.out.println(this.toString()+": received TAR_START_SYNC_PRED, but distanceToPredecessor = "
					+distanceToPredecessor+" >= D_Sync = "+this.D_Sync+", ignore the event.");
		}
	}
	/**
	 * @assume driving along X-axis.
	 */
	private void processTarStopSyncPred(Event event) {
		if (this.node != TargetlaneVehicleNodeType.SYNC) {
			System.out.println(this.toString()+": received TAR_STOP_SYNC_PRED, while node != SYNC, ignore event.");
			return; //ignore the event;
		}
		if (this.predecessor == null)
			throw new RuntimeException(this.toString()+": received TAR_STOP_SYNC_PRED, while precedessor = null.");
		this.node = TargetlaneVehicleNodeType.INIT;
		this.state = "Init";
		//correct the accumulated double computation errors.
		this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_CONSTANT_SPEED, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_lim(), 0, 0, 
				0, 0, 0);
		if (this.successor != null) {
			ArrayList<Entity> destinations = new ArrayList<Entity>();
			destinations.add(this.successor);
			Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.TAR_STOP_SYNC_PRED, this, destinations, "");
			Simulator.addEvent(nextEvent);
			System.out.println(this.toString()+": added "+nextEvent.toString());
		}
	}
	private void processGlobalSendLaneChangeReq(Event event) {
		if (this.node != TargetlaneVehicleNodeType.INIT) {
			System.out.println(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_REQ, while node != INIT, ignore event.");
			return; //ignore the event
		}
		if (Math.random() > this.per) { //The message did get through the wireless medium
			Entity source = event.getSource();
			if (source == null)
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_REQ, while source == null.");
			if (!(source instanceof VehicleR))
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_REQ, while source is not a VehicleR object.");
			this.R = (VehicleR) source;
			ArrayList<Entity> destinations = event.getDestinations();
			if (destinations == null || !destinations.contains(this))
				throw new RuntimeException(this.toString()+": received GLOBAL_SEND_LANE_CHANGE_REQ, but the destinations is null or does not contain me.");
			String content = event.getContent();
			StringTokenizer st = new StringTokenizer(content);
			this.d_0_R_L = Double.parseDouble(st.nextToken());
			this.d_0_R_F = Double.parseDouble(st.nextToken());
			//node transition
			this.node = TargetlaneVehicleNodeType.VALIDATION;
			//note node Validation is a transient node, the entrance to Validation immediately triggers the next event.
			//correct the accumulated double computation errors.
			this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_CONSTANT_SPEED, 
					this.get_X(), this.get_Y(), 0, 
					this.get_v_lim(), 0, 0, 
					0, 0, 0);
			ArrayList<Entity> nextEventDestination = new ArrayList<Entity>();
			nextEventDestination.add(this);
			Event nextEvent = null;
			if (this.d_0_R_L >= this.D_1 && this.d_0_R_F < this.D_3) {
				nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.TAR_EVENT1, this, nextEventDestination, "");
			} else if ((this.d_0_R_L < this.D_1 && this.d_0_R_F < this.D_8)) {
				nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.TAR_EVENT2, this, nextEventDestination, "");
			} else {
				nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.TAR_VALIDATION_FAILURE, this, nextEventDestination, "");
			}
			Simulator.addEvent(nextEvent);
			System.out.println(this.toString()+": added "+nextEvent.toString());
		} else {
			System.out.println(this.toString()+": GLOBAL_SEND_LANE_CHANGE_REQ happened, but lost due to PER, ignore event.");
		}
	}
	/**
	 * @assume driving along the X-axis.
	 */
	private void processTarEvent1(Event event) {
		if (this.node != TargetlaneVehicleNodeType.VALIDATION)
			throw new RuntimeException(this.toString()+": received TAR_EVENT1, while node != VALIDATION.");
		//send the LaneChangeAcpt message	
		ArrayList<Entity> destinations = new ArrayList<Entity>();
		if (this.R == null)
			throw new RuntimeException(this.toString()+": received TAR_EVENT1, while this.R == null.");
		if (!(this.R instanceof VehicleR))
			throw new RuntimeException(this.toString()+": received TAR_EVENT1, while this.R is not a VehicleR object.");
		destinations.add(this.R);
		double tmp = (this.D_3 + this.d_d_v_lim_v_low - this.get_v_low() * this.delta_d_v_lim_v_low_Tick * Simulator.getSingletonClock().getTickDuration() - this.d_0_R_F) 
				/ (this.get_v_lim() - this.get_v_low()); //unit: ISO
		this.delta_low_RF1_Tick = this.get_delta_lc_v_lim_Tick() +  (long)Math.round(tmp / Simulator.getSingletonClock().getTickDuration()) ;
		this.delta_defer_lc_Tick = this.delta_d_v_lim_v_low_Tick + (long)Math.round(tmp / Simulator.getSingletonClock().getTickDuration());
		Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.GLOBAL_SEND_LANE_CHANGE_ACPT, this, destinations, this.delta_defer_lc_Tick+"");
		Simulator.addEvent(nextEvent);
		System.out.println(this.toString()+": added "+nextEvent.toString());
		//node transition
		this.node = TargetlaneVehicleNodeType.DECELERATING_1;
		this.state = "Coop";
		this.tauTick = 0;
		this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_DECELERATE, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_lim(), 0, 0, 
				0, 0, 0);
		if (this.successor != null) {
			ArrayList<Entity> dest = new ArrayList<Entity>();
			dest.add(this.successor);
			Event nextEvt = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.TAR_START_SYNC_PRED, this, dest, "");
			Simulator.addEvent(nextEvt);
			System.out.println(this.toString()+": added "+nextEvt.toString());
		}
	}
	/**
	 * @assume driving along the X-axis.
	 * @assume TAR_EVENT2 is only triggered by GLOBAL_SEND_LANE_CHANGE_REQ under the condition of
	 *     (this.d_0_R_L < this.D_1 && this.d_0_R_F < this.D_8)
	 *     @see #processGlobalSendLaneChangeReq(Event).
	 */
	private void processTarEvent2(Event event) {
		if (this.node != TargetlaneVehicleNodeType.VALIDATION) 
			throw new RuntimeException(this.toString()+": received TAR_EVENT2, while node != VALIDATION.");
		if (this.d_0_R_L < this.D_1 && this.d_0_R_F < this.D_8) {
			double tmp1 = (this.get_v_lim() * this.Delta_star_Tick * Simulator.getSingletonClock().getTickDuration() - this.d_0_R_L) / (this.get_v_lim() - this.get_v_low()); //unit: ISO
			this.delta_low_LR_Tick = (long)Math.round(tmp1 / Simulator.getSingletonClock().getTickDuration());
			double tmp2 = (this.D_8 - this.d_0_R_F ) / (this.get_v_lim() - this.get_v_low()); //unit: ISO
			this.delta_low_RF2_Tick = (long)Math.round(tmp2 / Simulator.getSingletonClock().getTickDuration());
			this.delta_defer_Rdec_Tick = this.delta_low_RF2_Tick + this.delta_d_v_lim_v_low_Tick;	
		} else {
			throw new RuntimeException(this.toString()+": received TAR_EVENT2, while the following precondition does not hold: \n"
					+"(this.d_0_R_L < this.D_1 && this.d_0_R_F < this.D_8) \n"
					+" where this.d_0_R_L = "+this.d_0_R_L+"\nthis.D_8 = "+this.D_8+"\nthis.d_0_R_F = "+this.d_0_R_F+"\nthis.get_v_lim() = "+this.get_v_lim()
					+"\nthis.Delta_star_Tick = "+this.Delta_star_Tick+"\nthis.D_1 = "+this.D_1+"\n");
		}
		
		//send the Decelerate message
		ArrayList<Entity> destinations = new ArrayList<Entity>();
		if (this.R == null)
			throw new RuntimeException(this.toString()+": received TAR_EVENT2, while this.R == null.");
		if (!(this.R instanceof VehicleR))
			throw new RuntimeException(this.toString()+": received TAR_EVENT2, while this.R is not a VehicleR object.");
		destinations.add(this.R);
		Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
				Event.EventType.GLOBAL_SEND_DECELERATE, this, destinations, this.delta_defer_Rdec_Tick+" "+this.delta_low_LR_Tick);
		Simulator.addEvent(nextEvent);
		System.out.println(this.toString()+": added "+nextEvent.toString());
		
		//node transition
				this.node = TargetlaneVehicleNodeType.DECELERATING_2;
				this.state = "Coop";
				this.tauTick = 0;
				this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_DECELERATE, 
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						0, 0, 0);
				if (this.successor != null) {
					ArrayList<Entity> dest = new ArrayList<Entity>();
					dest.add(this.successor);
					Event nextEvt = new Event(Simulator.getSingletonClock().getCurrentTick(), 
							Event.EventType.TAR_START_SYNC_PRED, this, dest, "");
					Simulator.addEvent(nextEvt);
					System.out.println(this.toString()+": added "+nextEvt.toString());
				}
	}
	private void processTarValidationFailure(Event event) {
		if (this.node != TargetlaneVehicleNodeType.VALIDATION)
			throw new RuntimeException(this.toString()+": received TAR_VALIDATION_FAILURE, while node != VALIDATION.");
		//node transition
		this.node = TargetlaneVehicleNodeType.INIT;
		this.state = "Init";
		this.tauTick = 0;
		this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_CONSTANT_SPEED, this.get_X(), this.get_Y(), 0, this.get_v_lim(), 0, 0, 0, 0, 0);
	}
	
	
	private void processTEvent2(Event event) {
		if (this.node != TargetlaneVehicleNodeType.DECELERATING_2) {
			throw new RuntimeException(this.toString()+": received T_EVENT2, while node != DECELERATING_2.");
		}
		//node transition
		this.node = TargetlaneVehicleNodeType.CONST_LOW_SPEED_2;
		this.state = "Coop";
		this.tauTick = 0;
		this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_CONSTANT_SPEED, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_low(), 0, 0, 
				0, 0, 0);
	}
	
	private void processTEvent3(Event event) {
		if (this.node != TargetlaneVehicleNodeType.CONST_LOW_SPEED_2) {
			throw new RuntimeException(this.toString()+": received T_EVENT3, while node != CONST_LOW_SPEED_2.");
		}
		//node transition
		this.node = TargetlaneVehicleNodeType.ACCELERATING;
		this.state = "Coop";
		this.tauTick = 0;
		int autoDrivingGearIndex = this.calculateGearIndexUnderForwardAccelerateDrivingMode(this.get_v_low());
		double acceleration = this.get_a(this.get_v_low(), autoDrivingGearIndex);
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_ACCELERATE, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_low(), 0, 0, 
				acceleration, 0, 0);
	}
	
	private void processTEvent4(Event event) {
		if (this.node != TargetlaneVehicleNodeType.DECELERATING_1) {
			throw new RuntimeException(this.toString()+": received T_EVENT4, while node != DECELERATING_1.");
		}
		//node transition
		this.node = TargetlaneVehicleNodeType.CONST_LOW_SPEED_1;
		this.state = "Coop";
		this.tauTick = 0;
		this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_CONSTANT_SPEED, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_low(), 0, 0, 
				0, 0, 0);
    }
	
	private void processTEvent5(Event event) {
		if (this.node != TargetlaneVehicleNodeType.CONST_LOW_SPEED_1) {
			throw new RuntimeException(this.toString()+": received T_EVENT5, while node != CONST_LOW_SPEED_1.");
		}
		//node transition
		this.node = TargetlaneVehicleNodeType.ACCELERATING;
		this.state = "Coop";
		this.tauTick = 0;
		int autoDrivingGearIndex = this.calculateGearIndexUnderForwardAccelerateDrivingMode(this.get_v_low());
		double acceleration = this.get_a(this.get_v_low(), autoDrivingGearIndex);
		this.updateKineticStateWithDrivingModeChange(DrivingMode.FORWARD_ACCELERATE, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_low(), 0, 0, 
				acceleration, 0, 0);
	}
	
	private void processTEvent6(Event event) {
		if (this.node != TargetlaneVehicleNodeType.ACCELERATING) {
			throw new RuntimeException(this.toString()+": received T_EVENT6, while node != ACCELERATING.");
		}
		//node transition
		this.node = TargetlaneVehicleNodeType.INIT;
		this.state = "Init";
		//this.tauTick = this.tauTick; //strictly follow paper's diagram
		this.updateKineticStateWithDrivingModeChange(Vehicle.DrivingMode.FORWARD_CONSTANT_SPEED, 
				this.get_X(), this.get_Y(), 0, 
				this.get_v_lim(), 0, 0, 
				0, 0, 0);
		if (this.successor != null) {
			ArrayList<Entity> destinations = new ArrayList<Entity>();
			destinations.add(this.successor);
			Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
					Event.EventType.TAR_STOP_SYNC_PRED, this, destinations, "");
			Simulator.addEvent(nextEvent);
			System.out.println(this.toString()+": added "+nextEvent.toString());
		}
	}
	
	/**
	 * @assume driving along X-axis
	 */
	private void processGlobalTick(Event event) {
		if (Simulator.timeToLog()) System.out.println(this.toStringLong());
		if (this.node == TargetlaneVehicleNodeType.INIT) {
			if (this.state == null || !this.state.equals("Init"))
				throw new RuntimeException(this.toString()+": node = INIT, state == null || not \"Init\".");
			//dot_X == v_lim, @assume driving along X-axis
			if (Math.abs(this.get_dot_X() - this.get_v_lim()) > epsilon)
				throw new RuntimeException(this.toString()+": node = INIT, dot_X = "+this.get_dot_X()+", v_lim = "+this.get_v_lim());
			this.updateKineticStateWithoutDrivingModeChange();
		}else if (this.node == TargetlaneVehicleNodeType.VALIDATION) {
			throw new RuntimeException(this.toString()+": received GLOBAL_TICK while node = VALIDATION.");
		}else if (this.node == TargetlaneVehicleNodeType.DECELERATING_2) {
			if (this.state == null || !this.state.equals("Coop"))
				throw new RuntimeException(this.toString()+": node = DECELERATING_2, state == null || not \"Coop\".");
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happens right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			//create next event, @assume driving along X-axis
			if (this.get_dot_X() <= this.get_v_low()) { //each decrement to velocity may be too big,
											 //so do not use |velocity -vLow| <= epsilon to check.
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_low(), 0, 0, 
						this.get_ddot_X(), 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.T_EVENT2, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == TargetlaneVehicleNodeType.CONST_LOW_SPEED_2) {
			if (this.state == null || !this.state.equals("Coop"))
				throw new RuntimeException(this.toString()+": node = CONST_LOW_SPEED_2, state == null || not \"Coop\".");
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happens right after a triggering GLOBAL_TICK event.
			//dot_X == v_low, @assume driving along X-axis
			if (Math.abs(this.get_dot_X() - this.get_v_low()) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_LOW_SPEED_2, dot_X = "+this.get_dot_X()+", v_low = "+this.get_v_low());
			this.updateKineticStateWithoutDrivingModeChange();
			//create next event
			long tmp = this.delta_low_RF2_Tick;
			if (this.tauTick >= tmp) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_low(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.T_EVENT3, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == TargetlaneVehicleNodeType.DECELERATING_1) {
			if (this.state == null || !this.state.equals("Coop"))
				throw new RuntimeException(this.toString()+": node = DECELERATING_1, state == null || not \"Coop\".");
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happens right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			//create next event, @assume driving along X-axis
			if (this.get_dot_X() <= this.get_v_low()) { //each decrement to velocity may be too big,
											 //so do not use |velocity -vLow| <= epsilon to check.
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_low(), 0, 0, 
						this.get_ddot_X(), 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.T_EVENT4, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == TargetlaneVehicleNodeType.CONST_LOW_SPEED_1) {
			if (this.state == null || !this.state.equals("Coop"))
				throw new RuntimeException(this.toString()+": node = CONST_LOW_SPEED_1, state == null || not \"Coop\".");
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happens right after a triggering GLOBAL_TICK event.
			//dot_X == v_low, @assume driving along X-axis
			if (Math.abs(this.get_dot_X() - this.get_v_low()) > epsilon)
				throw new RuntimeException(this.toString()+": node = CONST_LOW_SPEED_1, dot_X = "+this.get_dot_X()+", v_low = "+this.get_v_low());
			this.updateKineticStateWithoutDrivingModeChange();
			//create next event
			long tmp = this.delta_low_RF1_Tick;
			if (this.tauTick >= tmp) {
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_low(), 0, 0, 
						0, 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.T_EVENT5, this, destinations, "");
				Simulator.addEvent(nextEvent);
				System.out.println(this.toString()+": added "+nextEvent.toString());
			}
		}else if (this.node == TargetlaneVehicleNodeType.ACCELERATING) {
			if (this.state == null || !this.state.equals("Coop"))
				throw new RuntimeException(this.toString()+": node = ACCELERATING, state == null || not \"Coop\".");
			this.tauTick++; //all events (particularly the node transition events) 
			//other than the CLOCK_INTERNAL_EVENT and the GLOBAL_TICK event happens right after a triggering GLOBAL_TICK event.
			this.updateKineticStateWithoutDrivingModeChange();
			//create next event, @assume driving along X-axis
			if (this.get_dot_X() >= this.get_v_lim()) { //each increment to velocity may be too big,
											  //so do not use |velocity - vLim| <= epsilon to check.
				//correct the accumulated double computation errors.
				this.updateKineticStateWithoutDrivingModeChange(
						this.get_X(), this.get_Y(), 0, 
						this.get_v_lim(), 0, 0, 
						this.get_ddot_X(), 0, 0);
				ArrayList<Entity> destinations = new ArrayList<Entity>();
				destinations.add(this);
				Event nextEvent = new Event(Simulator.getSingletonClock().getCurrentTick(), 
						Event.EventType.T_EVENT6, this, destinations, "");
				Simulator.addEvent(nextEvent);
			}
		}else if (this.node == TargetlaneVehicleNodeType.SYNC) {
			if (this.state == null || !this.state.equals("Sync"))
				throw new RuntimeException(this.toString()+": node = SYNC, state == null || not \"Sync\".");
			if (this.predecessor == null) throw new RuntimeException(this.toString()+": predecessor = null, while node = SYNC");
			//Because event destinations collection is a sorted array list (@see Event constructor parameter constraints), 
			//predecessor's state is updated by now.
			this.updateKineticStateWithoutDrivingModeChange(this.get_X(), this.get_Y(), this.get_psi(), 
					this.predecessor.get_dot_X(), this.predecessor.get_dot_Y(), this.predecessor.get_dot_psi(), 
					this.predecessor.get_ddot_X(), this.predecessor.get_ddot_Y(), this.predecessor.get_ddot_psi());
			//but just to keep the state consistent.
			if (this.get_dot_X() - this.get_v_lim() > epsilon) 
				throw new RuntimeException(this.toString()+": dot_X = "+this.get_dot_X()+" - v_lim = "+this.get_v_lim()+" exceeds epsilon = "+epsilon);
		}
	}
}
