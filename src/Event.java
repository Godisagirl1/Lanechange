import java.util.*;

public class Event {
	private long absoluteTick;
	public long getAbsoluteTick() { return this.absoluteTick; }
	public enum EventType{
		CLOCK_INTERNAL_TICK, GLOBAL_TICK, 
		
		//between vehicles' hybrid automata
		GLOBAL_SEND_LANE_CHANGE_REQ,
		GLOBAL_SEND_LANE_CHANGE_ACPT, GLOBAL_SEND_DECELERATE, 


		//in vehicle R's hybrid automaton
		//VR_XXX are those events used in proof (i.e. more important events).
		VR_EVENT1, VR_EVENT2, VR_EVENT3, VR_EVENT4, 
		//R_XXX are those events not used in proof (i.e. less important events).
		R_INTEND_CHANGE_LANE, R_ACTUAL_LANE_CHANGE, R_ACTUAL_DECELERATE,
		R_EVENT1_E3, R_EVENT1_E4, R_EVENT2, R_EVENT3, R_EVENT4, R_EVENT5, R_EVENT6,

		//in target lane vehicle's hybrid automaton
		//TAR_XXX are those events used in proof (i.e. more important events).
		TAR_EVENT1, TAR_EVENT2, 
		TAR_START_SYNC_PRED, TAR_STOP_SYNC_PRED, TAR_VALIDATION_FAILURE, 
		//T_XXX are those events not used in proof (i.e. less important events).
		T_EVENT2, T_EVENT3, T_EVENT4, T_EVENT5, T_EVENT6
	};
	private EventType type;
	public EventType getType() { return this.type; }
	public static String getTypeString(EventType type) {
		switch(type) {
		case CLOCK_INTERNAL_TICK: return "CLOCK_INTERNAL_TICK";
		case GLOBAL_TICK: return "GLOBAL_TICK";
		case GLOBAL_SEND_LANE_CHANGE_REQ: return "GLOBAL_SEND_LANE_CHANGE_REQ";
		case GLOBAL_SEND_LANE_CHANGE_ACPT: return "GLOBAL_SEND_LANE_CHANGE_ACPT";
		case GLOBAL_SEND_DECELERATE: return "GLOBAL_SEND_DECELERATE";
		case TAR_START_SYNC_PRED: return "TAR_START_SYNC_PRED"; 
		case TAR_STOP_SYNC_PRED: return "TAR_STOP_SYNC_PRED";
		case VR_EVENT1: return "VR_EVENT1";
		case VR_EVENT2: return "VR_EVENT2";
		case VR_EVENT3: return "VR_EVENT3";
		case VR_EVENT4: return "VR_EVENT4";
		case R_INTEND_CHANGE_LANE: return "R_INTEND_CHANGE_LANE";
		case R_ACTUAL_LANE_CHANGE: return "R_ACTUAL_LANE_CHANGE";
		case R_ACTUAL_DECELERATE: return "R_ACTUAL_DECELERATE";
		case R_EVENT1_E3: return "R_EVENT1_E3";
		case R_EVENT1_E4: return "R_EVENT1_E4";
		case R_EVENT2: return "R_EVENT2";
		case R_EVENT3: return "R_EVENT3";
		case R_EVENT4: return "R_EVENT4";
		case R_EVENT5: return "R_EVENT5";
		case R_EVENT6: return "R_EVENT6";
		case TAR_EVENT1: return "TAR_EVENT1";
		case TAR_EVENT2: return "TAR_EVENT2";
		case TAR_VALIDATION_FAILURE: return "TAR_VALIDATION_FAILURE";
		case T_EVENT2: return "T_EVENT2";
		case T_EVENT3: return "T_EVENT3";
		case T_EVENT4: return "T_EVENT4";
		case T_EVENT5: return "T_EVENT5";
		case T_EVENT6: return "T_EVENT6";
		}
		return "Wrong Event Type"+type;
	}
	private Entity source;
	public Entity getSource() { return this.source; }
	private ArrayList<Entity> destinations = new ArrayList<Entity>();
	public ArrayList<Entity> getDestinations() { return this.destinations; }
	private String content;
	public String getContent() { return this.content; }
	/**
	 * @param absoluteTick
	 * @param type
	 * @param source
	 * @param destinations @assume Because of the Sync node in HighwayVehicle, if several highway vehicles are involved, 
	 * they must be sorted in this destinations collection in ascending order of ids. This is to guarantee that the predecessor's 
	 * state is updated when we update a syncing successor highway vehicle. So as to achieve better accuracy 
	 * in the discrete simulation of the real world. This rule is bug prone that is better removed.
	 */
	public Event(long absoluteTick, EventType type, 
			Entity source, ArrayList<Entity> destinations) {
		this(absoluteTick, type, source, destinations, "");
	}
	/**
	 * @param absoluteTick
	 * @param type
	 * @param source
	 * @param destinations @assume Because of the Sync node in HighwayVehicle, if several highway vehicles are involved, 
	 * they must be sorted in this destinations collection in ascending order of ids. This is to guarantee that the predecessor's 
	 * state is updated when we update a syncing successor highway vehicle. So as to achieve better accuracy 
	 * in the discrete simulation of the real world. This rule is bug prone that is better removed.
	 * @param content
	 */
	public Event(long absoluteTick, EventType type, 
			Entity source, ArrayList<Entity> destinations, String content) {
		this.absoluteTick = absoluteTick;
		this.type = type;
		this.source = source;
		this.destinations = destinations;
		this.content = content;
	}
	public String toString() {
		StringBuffer destinationsString = new StringBuffer("[");
		for (int i = 0; i < this.destinations.size(); i++) {
			Entity destination = this.destinations.get(i);
			if (i != this.destinations.size() - 1) 
				destinationsString.append(destination.toString()+", ");
			else
				destinationsString.append(destination.toString());
		}
		destinationsString.append("]");
		
		return "[Event absoluteTick = "+this.absoluteTick
				+" type = "+getTypeString(this.type)
				+" source = "+this.source.toString()
				+" destinations = "+destinationsString.toString()
				+" content = "+this.content
				+"]";
	}
}
