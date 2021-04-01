import java.io.*;
import java.util.*;

public class ExtractTimeHeadway {
	/**
	 * Assume all .m trajectories are in gnuplot format, i.e. only data, no "xxx = [" nor "];".
	 */
	public static int getTargetlaneVehicleIdRightAfterR(int simulationId, int numberOfTargetlaneVehicles) {
		try{
			if (simulationId < 0)
				throw new RuntimeException("simulationId = "+simulationId+" < 0.");
			if (numberOfTargetlaneVehicles < 0) 
				throw new RuntimeException("number of targetlane vehicles = "+numberOfTargetlaneVehicles+" < 0.");
			String fileNamePrefix = "sim"+simulationId;
			double[] X_Tar = new double[numberOfTargetlaneVehicles];

			//extract the position of the vehicle R by the end of the simulation.
			String fileName = fileNamePrefix+"r.m";
			Scanner scanner = new Scanner(new BufferedReader(new FileReader(fileName)));
			String lastLine = "";
			while (scanner.hasNextLine()) {
				lastLine = scanner.nextLine();
			}
			scanner.close();
			StringTokenizer st = new StringTokenizer(lastLine);
			long tickOfR = Long.parseLong(st.nextToken());
			double X_R = Double.parseDouble(st.nextToken());  
      			double Y_R = Double.parseDouble(st.nextToken()); 

			//extract the position of all targetlane vehicles by the end of the simulation.
			for (int id = 0; id < X_Tar.length; id++) {
				fileName = fileNamePrefix+"tar"+id+".m";
				scanner = new Scanner(new BufferedReader(new FileReader(fileName)));
				lastLine = "";
				while (scanner.hasNextLine()) {
					lastLine = scanner.nextLine();
				}
				scanner.close();
				st = new StringTokenizer(lastLine);
				long tickOfTar = Long.parseLong(st.nextToken());
				X_Tar[id] = Double.parseDouble(st.nextToken());
				if (tickOfTar != tickOfR) 
					throw new RuntimeException("End of simulation at R is "+tickOfR+" ticks while at Tar"+id+" is "+tickOfTar+" ticks.");
			}
			//find result
			int result = -1;
			for (int id = 0; id < X_Tar.length; id++) {
				if (X_R >= X_Tar[id]) {
					result = id;
					break;
				}
			}
			return result;
		}catch(IOException exception) {
			exception.printStackTrace();
			throw new RuntimeException(exception.toString());
		}
	}
	/**
	 * 
	 * @param simulationId
	 */
	public static long getLanechangeTimeTick(int simulationId) {
		try{
			if (simulationId < 0)
				throw new RuntimeException("simulationId = "+simulationId+" < 0.");
			//extract the position of the vehicle R by the end of the simulation.
			String fileName = "sim"+simulationId+"r.m";
			Scanner scanner = new Scanner(new BufferedReader(new FileReader(fileName)));
			while (scanner.hasNextLine()) {
					String line = scanner.nextLine();
					StringTokenizer st = new StringTokenizer(line);
					long tick = Long.parseLong(st.nextToken());
					double X_R= Double.parseDouble(st.nextToken());
					double Y_R = Double.parseDouble(st.nextToken());
					double dot_X_R = Double.parseDouble(st.nextToken());
					if (Y_R >= 3.75 & dot_X_R >= 25) {
						scanner.close();
						return tick; //this is the time instance that R finishes the lanechanging.
					}
			}
			scanner.close();
			return -1;
		}catch(IOException exception) {
			exception.printStackTrace();
			throw new RuntimeException(exception.toString());
		}
	}
	public static void main(String args[]) {
		if (args == null || args.length != 3) {
			System.out.println("Usage: java ExtractTimeHeadway "
					+"<number of simulations> "
					+"<number of targetlane vehicles> "
					+"<sampling step size, sample once every so many trajectory logs, unit: logs/step>");
			System.exit(-1);
		}
		int numberOfSimulations = Integer.parseInt(args[0]);
		int numberOfTargetlaneVehicles = Integer.parseInt(args[1]);
		if (numberOfSimulations < 0)
			throw new RuntimeException("number of simulations = "+numberOfSimulations+" < 0.");
		if (numberOfTargetlaneVehicles < 0)
			throw new RuntimeException("number of targetlane vehicles = "+numberOfTargetlaneVehicles+" < 0.");
		long samplingStepSize = Long.parseLong(args[2]);
		
		try {
			double globalMinTimeHeadway = Double.POSITIVE_INFINITY;
			for (int simulationId = 0; simulationId < numberOfSimulations; simulationId++) {
				double simMinTimeHeadway = Double.POSITIVE_INFINITY;
				String fileNamePrefixInput = "sim"+simulationId;
				long tickLanechange = getLanechangeTimeTick(simulationId); //-1 means R never got lanechanging.
				int targetlaneVehicleIdRightAfterR = -1;
				if (tickLanechange != -1)
					targetlaneVehicleIdRightAfterR = getTargetlaneVehicleIdRightAfterR(simulationId, numberOfTargetlaneVehicles);
				String fileNameOutput = "sim"+simulationId+"_time_headways.m";
				PrintWriter pw = new PrintWriter(new FileWriter(fileNameOutput));
				System.out.println("Generating "+fileNameOutput+".");
				//extract the time headway for each targetlane vehicle
				for (int id = 0; id < numberOfTargetlaneVehicles; id++) {
					if (tickLanechange != -1 && id == 0 && id != targetlaneVehicleIdRightAfterR || tickLanechange == -1 && id == 0) {
					//do nothing, no predecessor.
					}else if (tickLanechange == -1 && id != 0) {
						Scanner scannerPre = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+(id-1)+".m")));
						Scanner scannerCur = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+id+".m")));
						long lineNumber = -1;
						while (scannerCur.hasNextLine()) {
							String linePre = scannerPre.nextLine();
							String lineCur = scannerCur.nextLine();
							lineNumber++;
							StringTokenizer stPre = new StringTokenizer(linePre);
							StringTokenizer stCur = new StringTokenizer(lineCur);
							long tickPre = Long.parseLong(stPre.nextToken());
							long tickCur = Long.parseLong(stCur.nextToken());
							if (tickPre != tickCur) {
								scannerCur.close();
								scannerPre.close();
								throw new RuntimeException("tar"+(id-1)+" and tar"+id+" trajectories unsynchronized at "
										+tickCur+" tick (as per tar"+id+").");
							}
							double X_Pre = Double.parseDouble(stPre.nextToken());
							double X_Cur = Double.parseDouble(stCur.nextToken());
							if (X_Pre <= X_Cur) {
								scannerCur.close();
								scannerPre.close();
								throw new RuntimeException("tar"+(id-1)+" at position "+X_Pre+" at "
										+tickCur+" tick while tar"+id+" at position "+X_Cur+".");
							}
							double Y_Pre = Double.parseDouble(stPre.nextToken());
							double Y_Cur = Double.parseDouble(stCur.nextToken());
							double dot_X_Cur = Double.parseDouble(stCur.nextToken());
							if (dot_X_Cur <= 0) {
								scannerCur.close();
								scannerPre.close();
								throw new RuntimeException("tar"+id+" velocity = "+dot_X_Cur+" m/s at "+tickCur+" tick.");
							}
							if (lineNumber % samplingStepSize == 0) {
									double timeHeadway = (X_Pre - X_Cur) / dot_X_Cur;
									pw.println(timeHeadway);
									if (timeHeadway < simMinTimeHeadway) simMinTimeHeadway = timeHeadway;
							}
						}
						scannerCur.close();
						scannerPre.close();
					}else if (tickLanechange != -1 && id == 0 && id == targetlaneVehicleIdRightAfterR) {
						Scanner scannerR = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"r.m")));
						Scanner scannerCur = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+id+".m")));
						long lineNumber = -1;
						while (scannerCur.hasNextLine()) {
							String lineR = scannerR.nextLine();
							String lineCur = scannerCur.nextLine();
							lineNumber++;
							StringTokenizer stR = new StringTokenizer(lineR);
							StringTokenizer stCur = new StringTokenizer(lineCur);
							long tickR = Long.parseLong(stR.nextToken());
							long tickCur = Long.parseLong(stCur.nextToken());
							if (tickR != tickCur) {
								scannerCur.close();
								scannerR.close();
								throw new RuntimeException("r and tar"+id+" trajectories unsynchronized at "
										+tickCur+" tick (as per tar"+id+").");
							}
							double X_R = Double.parseDouble(stR.nextToken());
							double X_Cur = Double.parseDouble(stCur.nextToken());
							if (tickR >= tickLanechange && X_R <= X_Cur) {
								scannerCur.close();
								scannerR.close();
								throw new RuntimeException("r at position "+X_R+" at "+tickR+" tick (tickLanechange = "
										+tickLanechange+") while tar"+id+" at position "+X_Cur+".");
							}
							double Y_R = Double.parseDouble(stR.nextToken());
							double Y_Cur = Double.parseDouble(stCur.nextToken());
							double dot_X_Cur = Double.parseDouble(stCur.nextToken());
							if (dot_X_Cur <= 0) {
								scannerCur.close();
								scannerR.close();
								throw new RuntimeException("tar"+id+" velocity = "+dot_X_Cur+" m/s at "+tickCur+" tick.");
							}
							if (tickR < tickLanechange) {
								//do nothing, as id == 0.
							}else {
								if (lineNumber % samplingStepSize == 0) {
									double timeHeadway = (X_R - X_Cur) / dot_X_Cur;
									pw.println(timeHeadway);
									if (timeHeadway < simMinTimeHeadway) simMinTimeHeadway = timeHeadway;
								}
							}
						}
						scannerCur.close();
						scannerR.close();
					}else if (tickLanechange != -1 && id > 0 && id == targetlaneVehicleIdRightAfterR) {
						Scanner scannerPre = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+(id-1)+".m")));
						Scanner scannerR = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"r.m")));
						Scanner scannerCur = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+id+".m")));
						long lineNumber = -1;
						while (scannerCur.hasNextLine()) {
							String linePre = scannerPre.nextLine();
							String lineR = scannerR.nextLine();
							String lineCur = scannerCur.nextLine();
							lineNumber++;
							StringTokenizer stPre = new StringTokenizer(linePre);
							StringTokenizer stR = new StringTokenizer(lineR);
							StringTokenizer stCur = new StringTokenizer(lineCur);
							long tickPre = Long.parseLong(stPre.nextToken());
							long tickR = Long.parseLong(stR.nextToken());
							long tickCur = Long.parseLong(stCur.nextToken());
							if (tickPre != tickCur) {
								scannerCur.close();
								scannerR.close();
								scannerPre.close();
								throw new RuntimeException("tar"+(id-1)+" and tar"+id+" trajectories unsynchronized at "
										+tickCur+" tick (as per tar"+id+").");
							}
							if (tickR != tickCur) {
								scannerCur.close();
								scannerR.close();
								scannerPre.close();
								throw new RuntimeException("r and tar"+id+" trajectories unsynchronized at "
										+tickCur+" tick (as per tar"+id+").");
							}
							double X_Pre = Double.parseDouble(stPre.nextToken());
							double X_R = Double.parseDouble(stR.nextToken());
							double X_Cur = Double.parseDouble(stCur.nextToken());
							if (X_Pre <= X_Cur) {
								scannerCur.close();
								scannerR.close();
								scannerPre.close();
								throw new RuntimeException("tar"+(id-1)+" at position "+X_Pre+" at "
										+tickCur+" tick while tar"+id+" at position "+X_Cur+".");
							}
							if (tickCur >= tickLanechange && X_R <= X_Cur) {
								scannerCur.close();
								scannerR.close();
								scannerPre.close();
								throw new RuntimeException("r at position "+X_R+" at "+tickCur+" tick (tickLanechange = "
										+tickLanechange+") while tar"+id+" at position "+X_Cur+".");
							}
							double Y_Pre = Double.parseDouble(stPre.nextToken());
							double Y_R = Double.parseDouble(stR.nextToken());
							double Y_Cur = Double.parseDouble(stCur.nextToken());
							double dot_X_Cur = Double.parseDouble(stCur.nextToken());
							if (dot_X_Cur <= 0) {
								scannerCur.close();
								scannerR.close();
								scannerPre.close();
								throw new RuntimeException("tar"+id+" velocity = "+dot_X_Cur+" m/s at "+tickCur+" tick.");
							}
							if (tickCur < tickLanechange) {
								if (lineNumber % samplingStepSize == 0) {
									double timeHeadway = (X_Pre - X_Cur) / dot_X_Cur;
									pw.println(timeHeadway);
									if (timeHeadway < simMinTimeHeadway) simMinTimeHeadway = timeHeadway;
								}
							}else {
								if (lineNumber % samplingStepSize == 0) {
									double timeHeadway = (X_R - X_Cur) / dot_X_Cur;
									pw.println(timeHeadway);
									if (timeHeadway < simMinTimeHeadway) simMinTimeHeadway = timeHeadway;
								}
							}
						}
						scannerR.close();
						scannerCur.close();
						scannerPre.close();
					}else if (tickLanechange != -1 && id > 0 && id != targetlaneVehicleIdRightAfterR) {
						Scanner scannerPre = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+(id-1)+".m")));
						Scanner scannerCur = new Scanner(
								new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+id+".m")));
						long lineNumber = -1;
						while (scannerCur.hasNextLine()) {
							String linePre = scannerPre.nextLine();
							String lineCur = scannerCur.nextLine();
							lineNumber++;
							StringTokenizer stPre = new StringTokenizer(linePre);
							StringTokenizer stCur = new StringTokenizer(lineCur);
							long tickPre = Long.parseLong(stPre.nextToken());
							long tickCur = Long.parseLong(stCur.nextToken());
							if (tickPre != tickCur) {
								scannerCur.close();
								scannerPre.close();
								throw new RuntimeException("tar"+(id-1)+" and tar"+id
										+" trajectories unsynchronized at "+tickCur+" tick (as per tar"+id+").");
							}
							double X_Pre = Double.parseDouble(stPre.nextToken());
							double X_Cur = Double.parseDouble(stCur.nextToken());
							if (X_Pre <= X_Cur) {
								scannerCur.close();
								scannerPre.close();
								throw new RuntimeException("tar"+(id-1)+" at position "+X_Pre
										+" at "+tickPre+" tick while tar"+id+" at position "+X_Cur+".");
							}
							double Y_Pre = Double.parseDouble(stPre.nextToken());
							double Y_Cur = Double.parseDouble(stCur.nextToken());
							double dot_X_Cur = Double.parseDouble(stCur.nextToken());
							if (dot_X_Cur <= 0) {
								scannerCur.close();
								scannerPre.close();
								throw new RuntimeException("tar"+id+" velocity = "+dot_X_Cur+" m/s at "+tickCur+" tick.");
							}
							if (lineNumber % samplingStepSize == 0) {
								double timeHeadway = (X_Pre - X_Cur) / dot_X_Cur;
								pw.println(timeHeadway);
								if (timeHeadway < simMinTimeHeadway) simMinTimeHeadway = timeHeadway;
							}
						}
						scannerCur.close();
						scannerPre.close();
					}
				}//end of for loop for each targetlane vehicle.
				//end of extracting time headway for each targetlane vehicle.
				//extracting time headway for the vehicle R.
				if (tickLanechange != -1 && numberOfTargetlaneVehicles <= 0) {//there is no targetlane vehicles
					//do nothing, no time headway to be computed.
				}else if (tickLanechange != -1 && targetlaneVehicleIdRightAfterR == 0) {//there are targetlane vehicles, and R has no predecessor targetlane vehicle.
					//do nothing, no time headway to be computed.
				}else if (tickLanechange != -1) {//there are targetlane vehicles, and R has a predecessor targetlane vehicle.
					int idPre = 0;
					if (targetlaneVehicleIdRightAfterR == -1) {
						idPre = numberOfTargetlaneVehicles - 1;
					}else {
						idPre = targetlaneVehicleIdRightAfterR - 1;
					}
					Scanner scannerPre = new Scanner(
							new BufferedReader(new FileReader(fileNamePrefixInput+"tar"+idPre+".m")));
					Scanner scannerR = new Scanner(
							new BufferedReader(new FileReader(fileNamePrefixInput+"r.m")));
					long lineNumber = -1;
					while (scannerR.hasNextLine()) {
						String linePre = scannerPre.nextLine();
						String lineR = scannerR.nextLine();
						lineNumber++;
						StringTokenizer stPre = new StringTokenizer(linePre);
						StringTokenizer stR = new StringTokenizer(lineR);
						long tickPre = Long.parseLong(stPre.nextToken());
						long tickR = Long.parseLong(stR.nextToken());
						if (tickPre != tickR) {
							scannerR.close();
							scannerPre.close();
							throw new RuntimeException("r and tar"+idPre+" trajectories unsynchronized at "
									+tickR+" tick (as per r).");
						}
						double X_Pre = Double.parseDouble(stPre.nextToken());
						double X_R = Double.parseDouble(stR.nextToken());
						if (tickR >= tickLanechange && X_Pre <= X_R) {
							scannerR.close();
							scannerPre.close();
							throw new RuntimeException("r at position "+X_R+" at "+tickR+" tick (tickLanechange = "
									+tickLanechange+") while tar"+idPre+" at position "+X_Pre+".");
						}
						if (tickR < tickLanechange) {
							//do nothing.
						}else {
							double Y_R = Double.parseDouble(stR.nextToken());
							double dot_X_R = Double.parseDouble(stR.nextToken());
							if (dot_X_R <= 0) {
								scannerR.close();
								scannerPre.close();
								throw new RuntimeException("r velocity = "+dot_X_R+" m/s at "+tickR+" tick.");
							}
							if (lineNumber % samplingStepSize == 0) {
								double timeHeadway = (X_Pre - X_R) / dot_X_R;
								pw.println(timeHeadway);
								if (timeHeadway < simMinTimeHeadway) simMinTimeHeadway = timeHeadway;
							}
						}
					}
					scannerPre.close();
					scannerR.close();
				}else { //tickLanechange == -1
					//do nothing
				}
				//end of extracting time headway for R.
				pw.flush();
				pw.close();
				System.out.println(fileNameOutput+" generated; min time headway for this simulation is "+simMinTimeHeadway+".");
				if (simMinTimeHeadway < globalMinTimeHeadway) globalMinTimeHeadway = simMinTimeHeadway;
			}//end of for loop for each simulation.
			System.out.println("Min time headway for all simulations is "+globalMinTimeHeadway+".");
		}catch(IOException e) {
			e.printStackTrace();
			throw new RuntimeException(e.toString());
		}
	}

}
