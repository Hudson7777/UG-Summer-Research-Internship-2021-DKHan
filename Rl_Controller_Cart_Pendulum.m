clear all;
close all;
clc;
%%
% PART1 : create environment 
% mdl = 'rl_haoran';
mdl = 'rlCartPoleSimscapeModel';
open_system(mdl)
%%
% create environment interface,including env object, obsInfo and actInfo
env = rlPredefinedEnv('CartPoleSimscapeModel-Continuous');
obsInfo = getObservationInfo(env);
numObservations = obsInfo.Dimension(1);
actInfo = getActionInfo(env);
actInfo.LowerLimit = -200;
actInfo.UpperLimit = 200;
% define ###reset function, simulation time and agent sample time
Ts = 0.02;
Tf = 25;
% fix the random generator seed
rng(0);

% PART2 : create DDPG agent
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(200,'Name','CriticStateFC2')];

actionPath = [
    featureInputLayer(1,'Normalization','none','Name','action')
    fullyConnectedLayer(200,'Name','CriticActionFC1','BiasLearnRateFactor',0)];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);    
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

% view the critic network 
figure
plot(criticNetwork)
% specify the options for critic representation
criticOptions = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);
% create critic representation
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);
% create the actor network, similar to critic network
actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(128,'Name','ActorFC1')
    reluLayer('Name','ActorRelu1')
    fullyConnectedLayer(200,'Name','ActorFC2')
    reluLayer('Name','ActorRelu2')
    fullyConnectedLayer(1,'Name','ActorFC3')
    tanhLayer('Name','ActorTanh1')
    scalingLayer('Name','ActorScaling','Scale',max(actInfo.UpperLimit))];

actorOptions = rlRepresentationOptions('LearnRate',5e-04,'GradientThreshold',1);

actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling'},actorOptions);
% specify DDPG agent options
agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'MiniBatchSize',128);
agentOptions.NoiseOptions.StandardDeviation = 0.4;
agentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-5;

% create DDPG agent
agent = rlDDPGAgent(actor,critic,agentOptions);

% PART3 ： train the agent
maxepisodes = 1000;
maxsteps = ceil(Tf/Ts);
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes,...
    'MaxStepsPerEpisode',maxsteps,...
    'ScoreAveragingWindowLength',5,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',-200,...
    'SaveAgentCriteria','EpisodeReward',...
    'SaveAgentValue',-400);

doTraining = false;

if doTraining    
    % Train the agent.
    trainingStats = train(agent,env,trainingOptions);
else
    % Load the pretrained agent for the example.
    load('Agent126.mat','saved_agent') % agent vs saved_agent
end

% save("/Users/yanyu/Desktop/Summer_Internship_2021/DK_Han/RL_controller/slprj/savedAgents/RL_Cart_Pendulum_agent.mat",'agent')

% PART4 : validate the trained agent
k =800;
simOptions = rlSimulationOptions('MaxSteps',k);

experience = sim(env,saved_agent,simOptions); % agent vs saved_agent
samples = getsamples(experience.Observation.observations,1:k);
data = samples.Data;
data_reshape = reshape(data,5,k);% 500 * 5 
data_final = data_reshape'; % 5 * 500
figure
plot(data_final);

% plot first obs info
%data_column1 = data_reshape;
%data_column1(:,2:3:4:5) = 0;
%plot(data_column1);

legend('sin theta','cos theta','x','d theta','d_x')
xlabel('Time Step'), ylabel('values')
title('Obeservation Infos')
% bdclose(mdl)