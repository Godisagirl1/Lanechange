import java.util.*;

public class Simulator {
	private static ArrayList<Event> theEventQueue = new ArrayList<Event>();
	private static boolean timeToStopSimulation = false;
	public static Clock getSingletonClock() { return Clock.getSingleton(); }
	private static VehicleR vehicleR;
	public static VehicleR getVehicleR() { return vehicleR; }
	private static ArrayList<TargetlaneVehicle> targetlaneVehicles = new ArrayList<TargetlaneVehicle>();
	public static TargetlaneVehicle getTargetlaneVehicle(int id) { return targetlaneVehicles.get(id); }
	public static ArrayList<TargetlaneVehicle> getTargetLaneVehicles() { return targetlaneVehicles; }
	private static ArrayList<Entity> allEntities = new ArrayList<Entity>();
	public static ArrayList<Entity> getAllEntities(){
		return allEntities;
	}
	private static ArrayList<Entity> allPhysicalEntities = new ArrayList<Entity>(); //allEntities minus Clock
	public static ArrayList<Entity> getAllPhysicalEntities(){
		return allPhysicalEntities;
	}
	/**
	 * logStepSizeMask = (1 << logStepSizeMaskSize) - 1
	 */
	private static int logStepSizeMaskSize = 0;
	/** 
	 * log the trajectory of vehicles every (logStepSizeMask+1) ticks, 
	 * logStepSizeMask = (1 << logStepSizeMaskSize) - 1.
	 */
	private static long logStepSizeMask = 0;
	public static boolean timeToLog() {
		return ((getSingletonClock().getCurrentTick() & logStepSizeMask) == 0);
	}
	
	public static void addEvent(Event event) {
		long eventTick = event.getAbsoluteTick();
		int i = 0;
		while (i < theEventQueue.size()) {
			Event tmpEvent = theEventQueue.get(i);
			long tmpTick = tmpEvent.getAbsoluteTick();
			if (eventTick < tmpTick)//i is the place
				break;
			else 
				i++; //move on. This means for events at the same singletonClock tick, the later inserted will be triggered later.
		}
		theEventQueue.add(i, event);
	}
	public static boolean allTargetlaneVehiclesStablized() {
		boolean result = true;
		for (int i = 0; i < targetlaneVehicles.size(); i++) {
			TargetlaneVehicle tar = targetlaneVehicles.get(i);
			if (!tar.isStablized()) {
				result = false;
				break;
			}
		}
		return result;
	}
	public static boolean allVehiclesStablized() {
		return (vehicleR.isStablized() && allTargetlaneVehiclesStablized());
	}
	public static void stopSimulation() {
		timeToStopSimulation = true;
	}
	private static double[] getDescendingInitialPositions(int numberOfVehicles, 
			double initialPositionDistributionRangeMin,
			double initialPositionDistributionRangeMax,
			long deltaStarTick,
			double durationOfATick, 
			double vLim) {
		if (numberOfVehicles < 0) throw new RuntimeException("numberOfVehicles = "+numberOfVehicles);
		double range = initialPositionDistributionRangeMax - initialPositionDistributionRangeMin;
		if (range < 0) throw new RuntimeException("initialPositionDistributionRangeMax = "
				+initialPositionDistributionRangeMax
				+", initialPositionDistributionRangeMin = "
				+initialPositionDistributionRangeMin);
		if (deltaStarTick <= 0)
			throw new RuntimeException("deltaStarTick = "+deltaStarTick);
		if (durationOfATick <= 0)
			throw new RuntimeException("durationOfATick = "+durationOfATick);
		if (vLim <= 0)
			throw new RuntimeException("vLim = "+vLim);
		double[] result = new double[numberOfVehicles];
		
		for (int i = 0; i < result.length; i++) {
			boolean needsRedo = false;
			do {
				result[i] = Math.random() * range + initialPositionDistributionRangeMin;
				double minDistanceToExistingPositions = Double.POSITIVE_INFINITY;
				for (int j = 0; j < i; j++) {
					double tmp = Math.abs(result[j] - result[i]);
					if (tmp <= minDistanceToExistingPositions)
						minDistanceToExistingPositions = tmp;
				}
				if (minDistanceToExistingPositions <= deltaStarTick * durationOfATick * vLim)
					needsRedo = true;
				else
					needsRedo = false;
			}while (needsRedo);
		}
		//sorting
		for (int i = 0; i < result.length-1; i++)
			for (int j = 0; j < result.length-i-1; j++)
				if (result[j] < result[j+1]) {//swap
					double tmp = result[j];
					result[j] = result[j+1];
					result[j+1] = tmp;
				}
		for (int i = 0; i < result.length-1; i++)
			if (result[i] < result[i+1])
				throw new RuntimeException("Sorting problem: result["+i+"] = "+result[i]+", result["+(i+1)+"] = "+result[i+1]);
		return result;
	}
	public static void main(String args[]) {
		if (args == null || args.length != 23) {
			System.out.println("Usage: java Simulator <0. duration of a singletonClock tick unit: s> \n"
					+"<1. n, num of targetlane vehicles at the beginning> \n"
					+"<2. targetlane vehicle initial position min unit:m> \n"
					+"<3. targetlane vehicle initial position max unit:m> \n"
					+"<4. per, the wireless packet error rate, a value in [0,1]> \n"
					+"<5. v_lim unit: m/s> \n"
					+"<6. v_low unit: m/s> \n"
					+"<7. vehicle R initial position unit: m> \n"
					+"<8. decelerationStage1DurationTick unit: tick> \n"
					+"<9. maxDecelerationMagnitude unit: m/s^2> \n"
					+"<10. delta_lc_v_lim_Tick, duration to lane change in v_lim, unit: tick> \n"
					+"<11. d_lc_v_lim, longitudial distance to lane change in v_lim, unit: m> \n"
					+"<12. delta_lc_v_low_Tick, duration to lane change in v_low, unit: tick> \n"
					+"<13. d_lc_v_low, longitudial distance to lane change in v_low, unit: m> \n"
					+"<14. Delta_star_Tick unit: tick> \n"
					+"<15. delta_a_v_low_v_lim_Tick, duration to accelerate from v_low to v_lim, unit: tick> \n"
					+"<16. d_a_v_low_v_lim, distance to accelerate from v_low to v_lim, unit: m> \n"
					+"<17. delta_d_v_lim_v_low_Tick, duration to decelerate from v_lim to v_low, unit: tick> \n"
					+"<18. d_d_v_lim_v_low, distance to decelerate from v_lim to v_low, unit: m> \n"
					+"<19. Delta_reset_Tick unit: tick> \n"
					+"<20. Delta_nonzeno_Tick unit: tick> \n"
					+"<21. logStepSizeMaskSize valid value: {0, 1, ..., 62}, log every 2^logStepSizeMaskSize ticks> \n"
					+"<22. endOfSimulationAbsoluteTick: tick> \n");
			System.exit(-1);
		}
		double durationOfATick = Double.parseDouble(args[0]);
		Simulator.getSingletonClock().setTickDuration(durationOfATick);
		allEntities.add(Simulator.getSingletonClock());
		System.out.println("Simulator got durationOfATick = "+Simulator.getSingletonClock().getTickDuration()+"s");
		int n = Integer.parseInt(args[1]);
		System.out.println("n = "+n);
		double targetlaneVehicleInitialPositionMin = Double.parseDouble(args[2]);
		System.out.println("targetlaneVehicleInitialPositionMin = "+targetlaneVehicleInitialPositionMin+"m");
		double targetlaneVehicleInitialPositionMax = Double.parseDouble(args[3]);
		System.out.println("targetlaneVehicleInitialPositionMax = "+targetlaneVehicleInitialPositionMax+"m");
		double per = Double.parseDouble(args[4]);
		System.out.println("per = "+per);
		double v_lim = Double.parseDouble(args[5]);
		System.out.println("v_lim = "+v_lim+"m/s");
		double v_low = Double.parseDouble(args[6]);
		System.out.println("v_low = "+v_low+"m/s");
		double vehicleRInitialPosition = Double.parseDouble(args[7]);
		System.out.println("vehicleRInitialPosition = "+vehicleRInitialPosition+"m");
		long decelerationStage1DurationTick = Long.parseLong(args[8]);
		System.out.println("decelerationStage1DurationTick = "+decelerationStage1DurationTick+"tick");
		double maxDecelerationMagnitude = Double.parseDouble(args[9]);
		System.out.println("maxDecelerationMagnitude = "+maxDecelerationMagnitude+"m/s^2");
		long delta_lc_v_lim_Tick = Long.parseLong(args[10]);
		System.out.println("delta_lc_v_lim_Tick = "+delta_lc_v_lim_Tick+"tick");
		double d_lc_v_lim = Double.parseDouble(args[11]);
		System.out.println("d_lc_v_lim = "+d_lc_v_lim+"m");
		long delta_lc_v_low_Tick = Long.parseLong(args[12]);
		System.out.println("delta_lc_v_low_Tick = "+delta_lc_v_low_Tick+"tick");
		double d_lc_v_low = Double.parseDouble(args[13]);
		System.out.println("d_lc_v_low = "+d_lc_v_low+"m");
		long Delta_star_Tick = Long.parseLong(args[14]);
		System.out.println("Delta_star_Tick = "+Delta_star_Tick+"tick");
		long delta_a_v_low_v_lim_Tick = Long.parseLong(args[15]);
		System.out.println("delta_a_v_low_v_lim_Tick = "+delta_a_v_low_v_lim_Tick+"tick");
		double d_a_v_low_v_lim = Double.parseDouble(args[16]);
		System.out.println("d_a_v_low_v_lim = "+d_a_v_low_v_lim+"m");
		long delta_d_v_lim_v_low_Tick = Long.parseLong(args[17]);
		System.out.println("delta_d_v_lim_v_lowTick = "+delta_d_v_lim_v_low_Tick+"tick");
		double d_d_v_lim_v_low = Double.parseDouble(args[18]);
		System.out.println("d_d_v_lim_v_low = "+d_d_v_lim_v_low+"m");
		long Delta_reset_Tick = Long.parseLong(args[19]);
		System.out.println("Delta_reset_Tick = "+Delta_reset_Tick+"tick");
		long Delta_nonzeno_Tick = Long.parseLong(args[20]);
		System.out.println("Delta_nonzeno_Tick = "+Delta_nonzeno_Tick+"tick");
		logStepSizeMaskSize = Integer.parseInt(args[21]);
		if (logStepSizeMask < 0 || logStepSizeMaskSize > 62) 
			throw new RuntimeException("logStepSize set to "+logStepSizeMaskSize+" not in {0, 1, ..., 62}.");
		System.out.println("logStepSizeMaskSize = "+logStepSizeMaskSize);
		logStepSizeMask = (1 << logStepSizeMaskSize) - 1;
		System.out.println("log step: "+(logStepSizeMask+1)+" tick/step");
		long endOfSimulationAbsoluteTick = Long.parseLong(args[22]);
		Simulator.getSingletonClock().setEndOfSimulationAbsoluteTick(endOfSimulationAbsoluteTick);
		System.out.println("end of simulation absolute tick = "+endOfSimulationAbsoluteTick+" tick.");
		
		//calculate dependent parameters
		double D_1 = v_lim * (Delta_star_Tick - delta_lc_v_lim_Tick ) * durationOfATick + d_lc_v_lim;
		if (D_1 <= 0) throw new RuntimeException("D_1 = "+D_1+" <= 0.");
		System.out.println("D_1 = "+D_1);
		double D_3 = v_lim * (Delta_star_Tick + delta_lc_v_lim_Tick ) * durationOfATick - d_lc_v_lim;
		if (D_3 <= 0) throw new RuntimeException("D_3 = "+D_3+" <= 0.");
		System.out.println("D_3 = "+D_3);
		double d_d_hat_v_lim = v_lim * delta_d_v_lim_v_low_Tick * durationOfATick - d_d_v_lim_v_low;
		double d_lc_hat_v_low = v_low* delta_lc_v_low_Tick * durationOfATick  - d_lc_v_low;
		double D_8 = 2 * v_lim * Delta_star_Tick * durationOfATick + d_d_hat_v_lim  + d_lc_hat_v_low + (v_lim - v_low) * (delta_lc_v_low_Tick + delta_a_v_low_v_lim_Tick) * durationOfATick;
		if (D_8 <= 0) throw new RuntimeException("D_8 = "+D_8+" <= 0.");
		System.out.println("D_8 = "+D_8);
		
		//checking constraint c1
		System.out.println("Checking constraint c1:");
		System.out.println("v_low = "+v_low+"m/s, v_lim = "+v_lim+"m/s");
		if (!((0 < v_low) && (v_low < v_lim))) throw new RuntimeException("Violation of c1: v_low = "+v_low+", v_lim = "+v_lim);
		System.out.println("Delta_nonzeno_Tick = "+Delta_nonzeno_Tick+"tick");
		if (Delta_nonzeno_Tick <= 0) throw new RuntimeException("Violation of c1: Delta_nonzeno_Tick = "+Delta_nonzeno_Tick+" <= 0.");
		System.out.println("Delta_star_Tick = "+Delta_star_Tick+"tick, delta_a_v_low_v_lim_Tick = "+delta_a_v_low_v_lim_Tick+"tick, d_a_v_low_v_lim = "+d_a_v_low_v_lim);
		if (!((Delta_star_Tick > delta_lc_v_low_Tick) && (delta_lc_v_low_Tick > delta_lc_v_lim_Tick) && (delta_lc_v_lim_Tick > delta_d_v_lim_v_low_Tick)))
			throw new RuntimeException("Violation of c1: Delta_star_Tick = "+Delta_star_Tick+", delta_lc_v_lim_Tick = "
				+delta_lc_v_lim_Tick+", delta_lc_v_low_Tick = "+delta_lc_v_low_Tick+", delta_d_v_lim_v_low_Tick = "+delta_d_v_lim_v_low_Tick);
		System.out.println("c1 check passed.");
		
		//checking constraint c2
		System.out.println("Checking constraint c2:");
		double d_d_hat_v_low = d_d_v_lim_v_low - v_low * delta_d_v_lim_v_low_Tick * durationOfATick;
		long Delta_Coop_Event1_max_Tick = delta_d_v_lim_v_low_Tick + delta_a_v_low_v_lim_Tick + delta_lc_v_lim_Tick + (long)Math.round((D_3 + d_d_hat_v_low)/ (v_lim - v_low) / durationOfATick);
		System.out.println("Delta_Coop_Event1_max_Tick = "+Delta_Coop_Event1_max_Tick+"tick");
		long Delta_Coop_Event2_max_Tick = delta_d_v_lim_v_low_Tick + delta_a_v_low_v_lim_Tick + (long)Math.round( D_8 / (v_lim - v_low) / durationOfATick);
		System.out.println("Delta_Coop_Event2_max_Tick = "+Delta_Coop_Event2_max_Tick+"tick");
		long Delta_Coop_max_Tick = Math.max(Delta_Coop_Event1_max_Tick, Delta_Coop_Event2_max_Tick);
		System.out.println("Delta_Coop_max_Tick = "+Delta_Coop_max_Tick+"tick");
		System.out.println("c2 check passed.");
		
		//checking constraint c3
		System.out.println("Checking constraint c3:");
		double D_Sync_Event1_min = v_lim * Delta_star_Tick * durationOfATick + D_3 + d_d_hat_v_lim +  d_d_hat_v_low  + (v_lim - v_low) * (delta_lc_v_lim_Tick + delta_a_v_low_v_lim_Tick) * durationOfATick;
		System.out.println("D_Sync_Event1_min = "+D_Sync_Event1_min);
		double D_Sync_Event2_min = v_lim * Delta_star_Tick * durationOfATick + d_d_hat_v_lim + D_8 + (v_lim - v_low) *delta_a_v_low_v_lim_Tick * durationOfATick;   	    	 
		System.out.println("D_Sync_Event2_min = "+D_Sync_Event2_min);
		double D_Sync = Math.max(D_Sync_Event1_min, D_Sync_Event2_min);
		System.out.println("D_Sync = "+D_Sync);
		System.out.println("c3 check passed.");

		
		//create target vehicles
		double[] targetlaneVehicleDescendingInitialPosition = getDescendingInitialPositions(n, 
				targetlaneVehicleInitialPositionMin,
				targetlaneVehicleInitialPositionMax,
				Delta_star_Tick, durationOfATick, v_lim);
		
		for (int i = 0; i < n; i++) {
			
			/*
			 * Class for targetlane vehicles.
			 * 
			 * @assume driving along X-axis
			 * 
			 * @param delta_lc_v_lim_Tick for $\delta_{lc}(v_{lim})$, i.e. lane change time cost if driving at $v_{lim}$, got via empirical tests or numerical simulations, unit: tick
			 * @param delta_lc_v_low_Tick for $\delta_{lc}(v_{low})$, i.e. lane change time cost if driving at $v_{low}$, got via empirical tests or numerical simulations, unit: tick
			 */
			/*public TargetlaneVehicle(
					//Parameters used by the parent class: Vehicle.
					double per, double v_lim, double v_low, double initialPosition, 
					long decelerationStage1DurationTick, double maxDecelerationMagnitude,
					long delta_lc_v_lim_Tick, long delta_lc_v_low_Tick,
					
					//Parameters to configure the TargetlaneVehicle hybrid automaton
					int id, 
					double D_1, double D_3, double D_8,
					double D_Sync, long Delta_star_Tick,
					double d_d_v_lim_v_low, long delta_d_v_lim_v_low_Tick) {*/

			TargetlaneVehicle tar = new TargetlaneVehicle(
					per /*wireless packet error rate*/, v_lim, v_low, targetlaneVehicleDescendingInitialPosition[i], 
					decelerationStage1DurationTick, maxDecelerationMagnitude,
					delta_lc_v_lim_Tick, delta_lc_v_low_Tick,
					
					i /*id*/, 
					D_1, D_3, D_8,
					D_Sync, Delta_star_Tick, 
					d_d_v_lim_v_low, delta_d_v_lim_v_low_Tick);
			targetlaneVehicles.add(tar);
			allEntities.add(tar);
			allPhysicalEntities.add(tar);
		}
		for (int i = 0; i < n; i++) {
			TargetlaneVehicle tar = targetlaneVehicles.get(i);
			if (i-1 >= 0) tar.setPredecessor(targetlaneVehicles.get(i-1));
			if (i+1 <= n-1) tar.setSuccessor(targetlaneVehicles.get(i+1));
		}
		
		//create vehicle R
		/**
		 * Class for the vehicle R.
		 * 
		 * @assume driving along X-axis
		 * 
		 * @param delta_lc_v_lim_Tick for $\delta_{lc}(v_{lim})$, i.e. lane change time cost if driving at $v_{lim}$, got via empirical tests or numerical simulations, unit: tick
		 * @param delta_lc_v_low_Tick for $\delta_{lc}(v_{low})$, i.e. lane change time cost if driving at $v_{low}$, got via empirical tests or numerical simulations, unit: tick
		 */
		/*public VehicleR(
				//Parameters used by the parent class: Vehicle.
				double per,	double v_lim, double v_low, double initialPosition, 
				long decelerationStage1DurationTick, double maxDecelerationMagnitude,
				long delta_lc_v_lim_Tick, long delta_lc_v_low_Tick,
				
				//Parameters to configure the VehicleR hybrid automaton
				double D_1, double D_3, double D_8,
				long Delta_nonzeno_Tick, long Delta_reset_Tick, long Delta_star_Tick) {*/
		vehicleR = new VehicleR(
				per /*wireless packet error rate*/, v_lim, v_low, vehicleRInitialPosition, 
				decelerationStage1DurationTick, maxDecelerationMagnitude,
				delta_lc_v_lim_Tick, delta_lc_v_low_Tick,
				
				D_1, D_3, D_8,
				Delta_nonzeno_Tick, Delta_reset_Tick, Delta_star_Tick);
		allEntities.add(vehicleR);
		allPhysicalEntities.add(vehicleR);
		
		//create clock, start clock
		ArrayList<Entity> theSetOfClock = new ArrayList<Entity>();
		theSetOfClock.add(Simulator.getSingletonClock());
		/* public Event(long absoluteTick, EventType type, 
			Entity source, ArrayList<Entity> destinations) */
		Event firstEvent = new Event(0, Event.EventType.CLOCK_INTERNAL_TICK, Simulator.getSingletonClock(), theSetOfClock);
		Simulator.addEvent(firstEvent);
		
		while ((!timeToStopSimulation) && (!theEventQueue.isEmpty())) {
			Event nextEvent = theEventQueue.remove(0);
			ArrayList<Entity> destinations = nextEvent.getDestinations();
			if (destinations == null)
				throw new RuntimeException("nextEvent destination = null.");
			for (int i = 0; i < destinations.size(); i++) {
				Entity entity = destinations.get(i);
				entity.process(nextEvent);
			}
		}
	}
}
