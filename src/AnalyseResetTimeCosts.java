import java.io.*;
import java.util.*;

public class AnalyseResetTimeCosts{
	public static void main(String args[]){
		if (args == null || args.length != 2){
			System.out.println("Usage: java AnalyseResetTimeCosts <starting simulation ID (inclusive)> <ending simulation ID (exclusive)>");
			System.exit(-1);
		}
		int startSimId = Integer.parseInt(args[0]);
		int endSimId = Integer.parseInt(args[1]);
		String fileName = "sim"+startSimId+"to"+endSimId+"reset_time_costs.m";
		try{
			Scanner scanner = new Scanner(new BufferedReader(new FileReader(fileName)));
			long min = Long.MAX_VALUE;
			long max = Long.MIN_VALUE;
			double sum = 0;
			ArrayList<Long> resetTimeCosts = new ArrayList<Long>();
			while (scanner.hasNextLine()){
				String line = scanner.nextLine();
				long time = Long.parseLong(line);
				resetTimeCosts.add(new Long(time));
				if (time < min) min = time;
				if (time > max) max = time;
				sum += time;
			}
			scanner.close();
			if (resetTimeCosts.size() == 0){
				//do nothing
			}else{
				System.out.println("Min reset time = "+min+" ticks.");
				System.out.println("Max reset time = "+max+" ticks.");
				double average = sum / resetTimeCosts.size();
				System.out.println("Average reset time = "+average+" ticks.");
				if (resetTimeCosts.size() == 1)
					System.out.println("Std = 0 tick.");
				else{
					sum = 0;
					for (int i = 0; i < resetTimeCosts.size(); i++){
						long time = resetTimeCosts.get(i).longValue();
						sum += (time - average) * (time - average);
					}
					double std = Math.sqrt(sum / (resetTimeCosts.size() - 1));
					System.out.println("Std reset time = "+std+" ticks.");
				}
				double median = 0;
				Collections.sort(resetTimeCosts);
				int size = resetTimeCosts.size();
				if (size % 2 == 1) {
					median = resetTimeCosts.get((size - 1) / 2);
				}else {
					median = (resetTimeCosts.get(size / 2 - 1) + resetTimeCosts.get(size/2) + 0.0) / 2;
				}
				System.out.println("Median reset time = "+median+" ticks.");
			}
		}catch(IOException e){
			e.printStackTrace();
			throw new RuntimeException(e.toString());
		}
	}
}

