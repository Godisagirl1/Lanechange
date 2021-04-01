public class test{
	public static void main(String args[]){
		System.out.println(Double.POSITIVE_INFINITY+" "+100.0);
		System.out.println(Double.NEGATIVE_INFINITY+" "+100.0);
		double v = Double.parseDouble(Double.POSITIVE_INFINITY+"");
		System.out.println("v = "+v);
		v = Double.parseDouble(Double.NEGATIVE_INFINITY+"");
		System.out.println("v = "+v);
	}
}
