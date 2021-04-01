all: bin/Simulator.class bin/ExtractTimeHeadway.class bin/ValidateResets.class bin/AnalyseLanechangingTimeCosts.class bin/AnalyseResetTimeCosts.class bin/AnalyseTimeHeadway.class

bin/Simulator.class: src/Clock.java src/CommunicationEntity.java src/Entity.java src/Event.java src/TargetlaneVehicle.java src/Listener.java src/VehicleR.java src/Simulator.java src/Vehicle.java
	javac -d bin -sourcepath src src/Simulator.java

bin/ExtractTimeHeadway.class: src/ExtractTimeHeadway.java
	javac -d bin -sourcepath src src/ExtractTimeHeadway.java

bin/ValidateResets.class : src/ValidateResets.java
	javac -d bin -sourcepath src src/ValidateResets.java
	
bin/AnalyseLanechangingTimeCosts.class: src/AnalyseLanechangingTimeCosts.java
	javac -d bin -sourcepath src src/AnalyseLanechangingTimeCosts.java

bin/AnalyseResetTimeCosts.class: src/AnalyseResetTimeCosts.java
	javac -d bin -sourcepath src src/AnalyseResetTimeCosts.java

bin/AnalyseTimeHeadway.class: src/AnalyseTimeHeadway.java
	javac -d bin -sourcepath src src/AnalyseTimeHeadway.java
