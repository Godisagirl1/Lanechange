public abstract class CommunicationEntity extends Entity{
	/** packet error rate */
	protected double per;
	//public double getPer() { return this.per; }
	
	public CommunicationEntity(double per) {
		if (!(0 <= per && per <= 1)) throw new RuntimeException("PER = "+per+" not in [0, 1]");
		this.per = per;
	}
}
