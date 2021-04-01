import java.util.*;

public class Clock extends Entity{
	public String toString() { return "[Clock at "+Simulator.getSingletonClock().getCurrentTick()+" tick]"; }
	public String getNodeString() { return "INIT"; } //clock has only one node.
	private double tickDuration = Double.MIN_VALUE;
	public double getTickDuration() { return this.tickDuration; }
	private long endOfSimulationAbsoluteTick = Long.MAX_VALUE;
	public void setEndOfSimulationAbsoluteTick(long endOfSimulationAbsoluteTick) {
		this.endOfSimulationAbsoluteTick = endOfSimulationAbsoluteTick;
	}
	/** Only the Simulator is allowed to call this method. */
	void setTickDuration(double tickDuration) {
		if (tickDuration > 0) {
			this.tickDuration = tickDuration;
		}else {
			throw new RuntimeException("tickDuration = "+tickDuration+" <= 0");
		}
	}
	private long currentTick = -1;
	public long getCurrentTick() { return this.currentTick; }
	
	private static Clock theSingleton = new Clock(Double.MIN_VALUE);
	private Clock(double tickDuration) {
		setTickDuration(tickDuration);
	}
	static Clock getSingleton() { return theSingleton; }
	/** Only the Simulator is allowed to call this method. */
	public void process(Event event) {
		if (event == null || event.getType() != Event.EventType.CLOCK_INTERNAL_TICK) {
			throw new RuntimeException("Clock got an event that is not CLOCK_INTERNAL_TICK, event type = "+((event == null)?"null":event.getType()));
		}
		this.currentTick = event.getAbsoluteTick();
		if (this.currentTick >= this.endOfSimulationAbsoluteTick) {
			System.out.println(this.toString()+": reached end of simulation absolute tick = "+endOfSimulationAbsoluteTick+", simulation terminated.");
			Simulator.stopSimulation();
			return;
		}
		//create next event: GLOBAL_TICK to Simulator.getAllPhysicalEntities()
		Event nextEvent = new Event (this.currentTick, Event.EventType.GLOBAL_TICK, this, Simulator.getAllPhysicalEntities());
		Simulator.addEvent(nextEvent);
		//create next event: CLOCK_INTERNAL_TICK to this
		ArrayList<Entity> theSetOfClock = new ArrayList<Entity>();
		theSetOfClock.add(this);
		nextEvent = new Event(this.currentTick+1, Event.EventType.CLOCK_INTERNAL_TICK, this, theSetOfClock);
		Simulator.addEvent(nextEvent);
	}

}
