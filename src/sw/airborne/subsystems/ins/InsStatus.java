package sw.airborne.subsystems.ins;

public enum InsStatus {
	INS_UNINIT(0), INS_RUNNING(1);
	private InsStatus(int value){
		this.value = value;
	}
	private int value;
}
