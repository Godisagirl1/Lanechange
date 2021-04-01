import java.io.*;
import java.util.*;

public class AnalyseLanechangingTimeCosts{
	public static void main(String args[]){
		if (args == null || args.length != 2){
			System.out.println("Usage: java AnalyseLanechangingTimeCosts <starting simulation ID (inclusive)> <ending simulation ID (exclusive)>");
			System.exit(-1);
		}
		int startSimId = Integer.parseInt(args[0]);
		int endSimId = Integer.parseInt(args[1]);
		String fileName = "sim"+startSimId+"to"+endSimId+"lanechanging_time_costs.m";
		try{
			Scanner scanner = new Scanner(new BufferedReader(new FileReader(fileName)));
			long min = Long.MAX_VALUE;
			long max = Long.MIN_VALUE;
			double sum = 0;
			ArrayList<Long> lanechangingTimeCosts = new ArrayList<Long>();
			while (scanner.hasNextLine()){
				String line = scanner.nextLine();
				long time = Long.parseLong(line);
				lanechangingTimeCosts.add(new Long(time));
				if (time < min) min = time;
				if (time > max) max = time;
				sum += time;
			}
			scanner.close();
			System.out.println("Number of lanechanging = "+lanechangingTimeCosts.size());
			if (lanechangingTimeCosts.size() == 0){
				//do nothing
			}else{
				System.out.println("Min lanechanging time = "+min+" ticks.");
				System.out.println("Max lanechanging time = "+max+" ticks.");
				double average = sum / lanechangingTimeCosts.size();
				System.out.println("Average lanechanging time = "+average+" ticks.");
				if (lanechangingTimeCosts.size() == 1)
					System.out.println("Std = 0 tick.");
				else{
					sum = 0;
					for (int i = 0; i < lanechangingTimeCosts.size(); i++){
						long time = lanechangingTimeCosts.get(i).longValue();
						sum += (time - average) * (time - average);
					}
					double std = Math.sqrt(sum / (lanechangingTimeCosts.size() - 1));
					System.out.println("Std lanechanging time = "+std+" ticks.");
				}
				double median = 0;
				Collections.sort(lanechangingTimeCosts);
				int size = lanechangingTimeCosts.size();
				if (size % 2 == 1) {
					median = lanechangingTimeCosts.get((size - 1) / 2);
				}else {
					median = (lanechangingTimeCosts.get(size / 2 - 1) + lanechangingTimeCosts.get(size/2) + 0.0)/2;
				}
				System.out.println("Median lanechanging time = "+median+" ticks.");
			}
		}catch(IOException e){
			e.printStackTrace();
			throw new RuntimeException(e.toString());
		}
	}
}

