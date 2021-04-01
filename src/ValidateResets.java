import java.io.*;
import java.util.*;

public class ValidateResets {
	public static void main(String args[]) {
		if (args == null || args.length != 2) {
			System.out.println("Usage: java ValidateResets "
					+"<starting simulation ID (inclusive)> "
					+"<ending simulation ID (exclusive)>");
			System.exit(-1);
		}
		int startingSimulationId = Integer.parseInt(args[0]);
		int endingSimulationId = Integer.parseInt(args[1]);
		int totalNumberOfSimulations = endingSimulationId - startingSimulationId;
		if (totalNumberOfSimulations < 0)
			throw new RuntimeException("Total number of simulations = "+totalNumberOfSimulations+".");
		ArrayList<Long> reset2TimeCosts = new ArrayList<Long>();
		
		try {
			int totalNumberOfSuccessfulLanechange = 0;
			for (int simId = startingSimulationId; simId < endingSimulationId; simId++) {
				String fileName = "sim"+simId+"reset_validation.m";
				Scanner scanner = new Scanner(new BufferedReader(new FileReader(fileName)));
				long lineNumber = -1;
				long curTimeTick = Long.MIN_VALUE;
				String curState = null;
				boolean reset2Happened = false;
				while (scanner.hasNextLine()) {
					lineNumber++;
					String line = scanner.nextLine();
					System.out.println(fileName+": "+line);
					StringTokenizer st = new StringTokenizer(line);
					String token1 = st.nextToken();
					if (token1 == null || !token1.equals("[R")) 
						throw new RuntimeException("line#"+lineNumber+": $1 == null or not \"[\"R'.");
					long newTimeTick = Long.parseLong(st.nextToken());
					System.out.println("newTimeTick = "+newTimeTick);
					if (newTimeTick > curTimeTick) {
						curTimeTick = newTimeTick;
					}else {
						throw new RuntimeException("line#"+lineNumber+": time tick not increasing.");
					}
					String newState = st.nextToken();
					System.out.println("newState = "+newState);
					if (newState == null)
						throw new RuntimeException("line#"+lineNumber+": state == null.");
					if (curState == null) {
						if (!newState.equals("starts")) {
							throw new RuntimeException("line#"+lineNumber+": must start with \"starts\".");
						}else {
							curState = newState;
						}
					}else { //curState != null
						if (curState.equals("starts")) {
							if (newState.equals("Reset")) {
								int typeOfReset = Integer.parseInt(st.nextToken());
								System.out.println("typeOfReset = "+typeOfReset);
								if (typeOfReset == 2) {
									if (reset2Happened) {
										throw new RuntimeException("line#"+lineNumber+": Reset 2 happened again!");
									}else {
										long time = Long.parseLong(st.nextToken());
										System.out.println("Found a Reset 2, time = "+time+".");
										reset2Happened = true;
										reset2TimeCosts.add(new Long(time));
									}	
								}else if (typeOfReset == 1) {
									//do nothing
								}else {
									throw new RuntimeException("line#"+lineNumber+": unknown type of reset.");
								}
								curState = newState;
							}else {
								throw new RuntimeException("line#"+lineNumber+": state != \"Reset\".");
							}
						}else if (curState.equals("Reset")) {
							if (newState.equals("starts")) {
								curState = newState;
							}else {
								throw new RuntimeException("line#"+lineNumber+": state != \"starts\".");
							}
						}else {
							throw new RuntimeException("line#"+lineNumber+": wrog curState content: \""+curState+"\".");
						}
					}
				}//end of processing the current line in the current output?.txt file.
				if (curState != null && curState.equals("starts"))
					throw new RuntimeException("The trajectory ends with an open \"starts\".");
				scanner.close();
			}//for loop of simIds.
			System.out.println(reset2TimeCosts.size()+" successful lanechange out of "+totalNumberOfSimulations+" simulations.");
			if (reset2TimeCosts.size() != 0) {
				long min = Long.MAX_VALUE;
				long max = Long.MIN_VALUE;
				double sum = 0;
				double median = 0;
				double average = 0;
				double std = 0;
				for (int i = 0; i < reset2TimeCosts.size(); i++) {
					long datum = reset2TimeCosts.get(i).longValue();
					sum += datum;
					if (datum < min) min = datum;
					if (datum > max) max = datum;
				}
				average = sum / reset2TimeCosts.size();
				if (reset2TimeCosts.size() == 1) std = 0;
				else {
					sum = 0;
					for (int i = 0; i < reset2TimeCosts.size(); i++) {
						long datum = reset2TimeCosts.get(i).longValue();
						sum += (datum - average) * (datum - average);
					}
					std = Math.sqrt(sum / (reset2TimeCosts.size() - 1));
				}
				Collections.sort(reset2TimeCosts);
				int size = reset2TimeCosts.size();
				if (size % 2 == 1) {
					median = reset2TimeCosts.get((size - 1) / 2);
				}else {
					median = (reset2TimeCosts.get(size / 2 - 1) + reset2TimeCosts.get(size/2) + 0.0)/2;
				}
				System.out.println("min reset 2 time cost: "+min+" ticks.");
				System.out.println("max reset 2 time cost: "+max+" ticks.");
				System.out.println("median reset 2 time cost: "+median+" ticks.");		
				System.out.println("average reset 2 time cost: "+average+" ticks.");
				System.out.println("std reset 2 time cost: "+std+" ticks.");
			}
		}catch(IOException e) {
			e.printStackTrace();
			throw new RuntimeException(e.toString());
		}
	}

}
