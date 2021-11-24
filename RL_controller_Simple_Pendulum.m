% PART1 : create environment 
mdl = 'rlSimplePendulumModel';
open_system(mdl)
% create environment interface,including env object, obsInfo and actInfo
env= rlPredefinedEnv('SimplePendulumModel-Continuous');
obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
numObs = 3;
% define reset function, simulation time and agent sample time
env.ResetFcn = @(in)setVariable(in,'theta0',pi,'Workspace',mdl);
Ts = 0.1;
Tf = 40;
% fix the random generator seed
rng(0);

% PART2 : create DDPG agent
statePath = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(600,'Name','CriticStateFC1')
    reluLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(300,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(1,'Normalization','none','Name','action')
    fullyConnectedLayer(300,'Name','CriticActionFC1','BiasLearnRateFactor',0)];
commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

% view the critic network 
figure
plot(criticNetwork)

% specify the options for critic representation
criticOpts = rlRepresentationOptions('LearnRate',1e-02,'GradientThreshold',1);

% create critic representation
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,'Observation',{'observation'},'Action',{'action'},criticOpts);

% create the actor network, similar to critic network
actorNetwork = [
    featureInputLayer(numObs,'Normalization','none','Name','observation')
    fullyConnectedLayer(600,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(300,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(1,'Name','ActorFC3')
    tanhLayer('Name','ActorTanh')
    scalingLayer('Name','ActorScaling','Scale',max(actInfo.UpperLimit))];

actorOpts = rlRepresentationOptions('LearnRate',1e-02,'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,'Observation',{'observation'},'Action',{'ActorScaling'},actorOpts);

% specify DDPG agent options
agentOpts = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'DiscountFactor',1.0, ...
    'MiniBatchSize',64, ...
    'ExperienceBufferLength',10000); 
agentOpts.NoiseOptions.Variance = 0.3;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;

% create DDPG agent
agent = rlDDPGAgent(actor,critic,agentOpts);

% PART3 ： train the agent
maxepisodes = 200;
maxsteps = ceil(Tf/Ts);
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes, ...
    'MaxStepsPerEpisode',maxsteps, ...
    'ScoreAveragingWindowLength',20, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',1000);

doTraining = true;

if doTraining
    % Train the agent.
    trainingStats = train(agent,env,trainOpts);
else
    % Load the pretrained agent for the example.
    % load
end

% save("/Users/abhijit/Desktop/GIT_Projects/Matlab/sldemo_househeatExample/slprj/savedAgents/Heat_agent.mat",'agent')

% PART4 : validate the trained agent
simOptions = rlSimulationOptions('MaxSteps',500);
experience = sim(env,agent,simOptions);

