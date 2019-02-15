package frc.robot.auton.pathfollowing.util;

/*
public class CommandPoset extends Command {

    enum addedType 
    {
        SEQUENTIAL, 
        PARALLEL
    }
    private int numCommands; //This is the total number of commands in the commandGroup
    private Command[] commandsVector; //This contains all of the commands in the commandGroup, each command being identifiable by its index here
    private int[] statesVector; //-1 for yet to run, 0 for running, 1 for finished
    private int[] prevStatesVector = null;
    private List<Integer> yetToRunIndices;
    private List<Integer> runningIndices;
    private List<Integer> haveRanIndices;
    private int[][] dependencyMatrix; //This is a skew-symmetric Matrix w/ [i][j] being 1 if i requires j finishes, -1 if j requires i finishes, and 0 if they do not require each other
    private double[] timeoutVector; //This is a vector of Timeouts, each being -1 in the abscence of a timeout
    private List<Adder> addedVector = new ArrayList<Adder>();

    public CommandPoset() {
    }
  
    public CommandPoset(String name) {
      super(name);
    }

  
    public final synchronized void addSequential(Command cmd){
        // Command.validate("Can not add new command to command group");
        addedVector.add(new Adder(cmd, -1, addedType.SEQUENTIAL));
    }

    public final synchronized void addSequential(Command cmd, double timeout){
        // validate("Can not add new command to command group");
        addedVector.add(new Adder(cmd, timeout, addedType.SEQUENTIAL));
    }

    public final synchronized void addParallel(Command cmd){
        // validate("Can not add new command to command group");
        addedVector.add(new Adder(cmd, -1, addedType.PARALLEL));
    }

    public final synchronized void addParallel(Command cmd, double timeout){
        // validate("Can not add new command to command group");
        addedVector.add(new Adder(cmd, timeout, addedType.PARALLEL));
    }

    public void addEdgesFromSeqsAndPars(Poset poset, List<Integer> sequentialIndices, List<Integer> parallelIndices){
        int num = sequentialIndices.size() + parallelIndices.size();
        List<Integer> chunksList = new ArrayList<Integer>();
        int prevSeq = -1 ;
        for (int count = 0; count<num; count++){
            if (sequentialIndices.contains(count)){
                chunksList.add(count - prevSeq);
            }
        }
        int chunksNum = chunksList.size();
        for (int chunkCount = 0; chunkCount<chunksNum -1; chunkCount++){
            for (int prevChunkIndexer = 0; prevChunkIndexer < chunksList.get(chunkCount); prevChunkIndexer++){
                for (int newChunkIndexer = 0; prevChunkIndexer < chunksList.get(chunkCount+1); prevChunkIndexer++){
                    poset.addEdge(poset.getNode(sumList(chunksList, 0, chunkCount-1)+newChunkIndexer), poset.getNode(sumList(chunksList, 0, chunkCount-2)+prevChunkIndexer));
                }
            }
        }
    }

    public int sumList(List<Integer> listo, int start, int stop){
        int tot = 0;
        for (int ind=start; ind<=stop; ind++){
            tot+=listo.get(ind);
        }
        return tot;
    }

    //add implementation
    public final void generateInfoFromAddedVector(){
        numCommands = addedVector.size();
        commandsVector = new Command[numCommands];
        statesVector = new int[numCommands];
        prevStatesVector = new int[numCommands];
        dependencyMatrix = new int[numCommands][numCommands];
        timeoutVector = new double[numCommands];
        Poset _poset = new Poset(numCommands);
        List<Integer> seqIndices = new ArrayList<Integer>();
        List<Integer> parIndices = new ArrayList<Integer>();
        for (int ind = 1; ind< numCommands; ind++ ){
            prevStatesVector[ind] = -2;
            statesVector[ind] = -1;
            commandsVector[ind] = addedVector.get(ind).getCommand();
            timeoutVector[ind] = addedVector.get(ind).getTimeOut();
            addedType type = addedVector.get(ind).getType();
            switch (type){
                case SEQUENTIAL:
                    seqIndices.add(ind);
                case PARALLEL:
                    parIndices.add(ind);
            }
        }
        addEdgesFromSeqsAndPars(_poset, seqIndices, parIndices);
        dependencyMatrix = _poset.getSkewSymmetricLinearForm();
    }


    public void updateIndexTypeIdentificationLists(){
        yetToRunIndices.clear();
        runningIndices.clear();
        haveRanIndices.clear();
        for (int ind = 0; ind< numCommands; ind++){
            if (statesVector[ind] == -1) {
                yetToRunIndices.add(ind);
            } else if (statesVector[ind] == 0){
                runningIndices.add(ind);
            } else {
                haveRanIndices.add(ind);
            }
        }
    }

    public boolean goodToRun(int commandIndex){
        for (int otherInd = 0; otherInd < numCommands; otherInd++){
            if (dependencyMatrix[commandIndex][otherInd] == 1 && statesVector[otherInd] != 1){
                return false;
            }
        }
        return true;
    }

    public boolean statesVectorChangedQ(){
        boolean ans = (prevStatesVector == statesVector);
        prevStatesVector = statesVector;
        return ans;
    }

    public boolean timedOutQ(int commandIndex){
        if (timeoutVector[commandIndex] == -1){
            return false;
        } else {
            double time = commandsVector[commandIndex].timeSinceIntialized();
            return !(time == 0 || time<timeoutVector[commandIndex]);
        }
    }

    public void handleYetToRun(){
        if  (! statesVectorChangedQ()){
            return;
        } else {
            for(int cmdInd : yetToRunIndices) {
                if (goodToRun(cmdInd)){
                    commandsVector[cmdInd].start();
                    statesVector[cmdInd] = 0;
                }
            }
        }
    }

    public void handleRunning(){
        for (int commandIndex = 0; commandIndex<numCommands; commandIndex++){
            if (timedOutQ(commandIndex)) {
                commandsVector[commandIndex]._cancel();
                statesVector[commandIndex] = 1;
              } //Note: The cmd.run() class returns if has not finished, while running execute
              if (! commandsVector[commandIndex].run()) {
                commandsVector[commandIndex].run().removed();
                statesVector[commandIndex] = 1;
              } 
        }
    }


    @Override
    void _initialize() {
        generateInfoFromAddedVector();
    }

    @Override
    void _execute(){
        handleYetToRun();
        handleRunning();
    }

    @Override
    void _end(){
        for (int cmdInd : yetToRunIndices){
            commandsVector[cmdInd]._cancel();
        }
        for (int cmdInd : runningIndices){
            commandsVector[cmdInd]._cancel();
        }
        for (int i = 0; i< numCommands; i++){
            statesVector[i]=1;
        }
    }

    @Override
  void _interrupted() {
    _end();
  }

  @Override
  protected boolean isFinished() {
    for (int i=0; i<numCommands; i++){
        if (statesVector[i] != 1){
            return false;
        }
    }
    return true;
  }

  @Override
  protected void initialize() {
  }


  @Override
  protected void execute() {
  }


  @Override
  protected void end() {
  }


  @Override
  protected void interrupted() {
  }

  public synchronized boolean isInterruptible(){
    if (!super.isInterruptible()) {
        return false;
    } else{
        for (int ind = 0; ind<numCommands; ind++){
            if (! commandsVector[ind].isInterruptible()){
                return false;
            } 

        return true;


    }
}

    

    
        
  }
        final class Adder {
        private final Command _command;
        private final double _timeout;
        private final addedType _type;


    
        public Adder(Command command, double timeOut, addedType type) {
          _command = command;
          _timeout = timeOut;
          _type = type;
          }

        public Command getCommand(){
            return _command;
        }

        public double getTimeOut(){
            return _timeout;
        }

        public addedType getType(){
            return _type;
        }
    
      }
        

}

*/


