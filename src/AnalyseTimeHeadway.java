import java.io.*;
import java.util.*;

public class AnalyseTimeHeadway{
	public static void main(String args[]){
		if (args == null || args.length != 2){
			System.out.println("Usage: java AnalyseTimeHeadway <starting simulation ID (inclusive)> <ending simulation ID (exclusive)>");
			System.exit(-1);
		}
		int startSimId = Integer.parseInt(args[0]);
		int endSimId = Integer.parseInt(args[1]);
		String fileName = "sim"+startSimId+"to"+endSimId+"_time_headways.m";
		try{
			Scanner scanner = new Scanner(new BufferedReader(new FileReader(fileName)));
			double min = Double.MAX_VALUE;
			double max = Double.MIN_VALUE;
			double sum = 0;
			ArrayList<Double> timeHeadways = new ArrayList<Double>();
			while (scanner.hasNextLine()){
				String line = scanner.nextLine();
				double time = Double.parseDouble(line);
				timeHeadways.add(new Double(time));
				if (time < min) min = time;
				if (time > max) max = time;
				sum += time;
			}
			scanner.close();
			if (timeHeadways.size() == 0){
				//do nothing
			}else{
				System.out.println("Min time headway = "+min+" s.");
				System.out.println("Max time headway = "+max+" s.");
				double average = sum / timeHeadways.size();
				System.out.println("Average time headway = "+average+" s.");
				if (timeHeadways.size() == 1)
					System.out.println("Std = 0 s.");
				else{
					sum = 0;
					for (int i = 0; i < timeHeadways.size(); i++){
						double time = timeHeadways.get(i).doubleValue();
						sum += (time - average) * (time - average);
					}
					double std = Math.sqrt(sum / (timeHeadways.size() - 1));
					System.out.println("Std time headway = "+std+" s.");
				}
				double median = 0;
				Collections.sort(timeHeadways);
				int size = timeHeadways.size();
				if (size % 2 == 1) {
					median = timeHeadways.get((size - 1) / 2);
				}else {
					median = (timeHeadways.get(size / 2 - 1) + timeHeadways.get(size/2) + 0.0)/2;
				}
				System.out.println("Median time headway = "+median+" s.");
			}
		}catch(IOException e){
			e.printStackTrace();
			throw new RuntimeException(e.toString());
		}
	}
}

