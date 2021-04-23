#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <openrave/openrave.h>
#include <openrave/planningutils.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <random>
#include "RRT.hpp"
using namespace OpenRAVE;
using namespace std;

std::vector<GraphHandlePtr> handles;

vector<dReal> operator+(const vector<dReal> & c1, const vector<dReal> & c2)
{
    vector<dReal> sumC;
    for (int i = 0; i < c1.size(); i++){
        sumC.push_back(c1[i] + c2[i]);
    }
    return sumC;
}

vector<dReal> operator-(const vector<dReal> & c1, const vector<dReal> & c2)
{
    vector<dReal> subC;
    for (int i = 0; i < c1.size(); i++){
        subC.push_back(c1[i] - c2[i]);
    }
    return subC;
}

vector<dReal> operator*(const vector<dReal> & c1, const dReal& mul)
{
    vector<dReal> mulC;
    for (int i = 0; i < c1.size(); i++){
        mulC.push_back(c1[i]*mul);
    }
    return mulC;
}

class RRTModule : public ModuleBase
{
public:
    RRTModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&RRTModule::MyCommand,this,_1,_2),
                        "This is an example command");
        RegisterCommand("passEnvRobot",boost::bind(&RRTModule::passEnvRobot,this,_1,_2),
                        "This is to pass env into the cpp here");
        RegisterCommand("setStartConfig",boost::bind(&RRTModule::setStartConfig,this,_1,_2),
                        "This is to set startConfig");
        RegisterCommand("setGoalConfig",boost::bind(&RRTModule::setGoalConfig,this,_1,_2),
                        "This is to pass goal to the RRT");
        RegisterCommand("setGoalBias",boost::bind(&RRTModule::setGoalBias,this,_1,_2),
                        "This is to set goal bias variable");
        RegisterCommand("getRRTPath",boost::bind(&RRTModule::getRRTPath,this,_1,_2),
                        "This is to excute RRT to get the path");
        
        
    }
    virtual ~RRTModule() {}
    
    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << "output";
        return true;
    }

    bool setStartConfig(std::ostream& sout, std::istream& sinput)
    {
        while (sinput) {
            std::string str;
            sinput >> str;
            if (str.empty()) continue;
            _startConfig.push_back(static_cast<dReal>(stod(str)));
        }
        return true;
    }

    bool setGoalConfig(std::ostream& sout, std::istream& sinput)
    {
        while (sinput) {
            std::string str;
            sinput >> str;
            if (str.empty()) continue;
            _goalConfig.push_back(static_cast<dReal>(stod(str)));
        }
        return true;
    }

    bool setGoalBias(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        _goalBias = static_cast<dReal>(stod(input));
        sout << "goalbais" << _goalBias;
        return true;
    }

    bool passEnvRobot(ostream& sout, istream& sinput)
    {
        _env = GetEnv();
        vector<RobotBasePtr> robots;
        _env->GetRobots(robots);
        _robot = robots[0];
        // _robot->GetActiveDOFLimits(_lowerLimits, _upperLimits);
        _activeJointNum = _robot->GetActiveDOF();
        // _lowerLimits = {-0.564602,-0.3536,-2.12131,-0.650001,-M_PI,-2.00001,-M_PI}; // this not work?
        // _upperLimits = {2.13539,   1.2963,-0.15,        3.75, M_PI,    -0.1, M_PI};
        _lowerLimits.clear();
        _upperLimits.clear();
        dReal lowerLimits[] = {-0.564602,-0.3536,-2.12131,-0.650001,-M_PI,-2.00001,-M_PI};
        dReal upperLimits[] = {2.13539,   1.2963,-0.15,        3.75, M_PI,    -0.1, M_PI};
        for (auto n:lowerLimits) _lowerLimits.push_back(n);
        for (auto n:upperLimits) _upperLimits.push_back(n);
        // cout << "lower" <<endl;
        // for (auto it = _lowerLimits.begin(); it != _lowerLimits.end(); it++)
        //     std::cout << *it << " ";
        // std::cout << std::endl;
        // cout << "upper" <<endl;
        // for (auto it = _upperLimits.begin(); it != _upperLimits.end(); it++)
        //     std::cout << *it << " ";
        // std::cout << std::endl;
        return true;
    }

    // double fRand(double fMin, double fMax)
    // {
    //     double f = (double)rand() / RAND_MAX;
    //     return fMin + f * (fMax - fMin);
    // }
    vector<dReal> randomGenCon()
    {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> dis(0, 1);//uniform distribution between 0 and 1
        dReal ran = dis(gen);

        vector<dReal> randomCon(_activeJointNum);

        if (ran > _goalBias){
            for (int i = 0; i < _activeJointNum; i++) {
                uniform_real_distribution<> dis(_lowerLimits[i], _upperLimits[i]);//uniform distribution between 0 and 1
                randomCon[i] = dis(gen);
            }
        } else {
            return _goalConfig;
        }
        return randomCon;
    }

    bool goalCheck(const vector<dReal>& config, RRTNode* randNode)
    {
        RRTNode* currNode = new RRTNode(config);
        dReal dis = currNode->dist(randNode);
        delete currNode;
        return dis <= _stepSize;
    }

    bool checkColl(const vector<dReal>& config)
    {
        _robot->SetActiveDOFValues(config);
        bool res =_env->CheckCollision(_robot) || _robot->CheckSelfCollision();
        return res;
    
    }

    RRTNode* expandTree(RRTNode* randNode)
    {   
        // find the nearestNode
        RRTNode* nearestNode = _tree.findnearestNei(randNode);

        // calculate the direction for expand
        vector<dReal> dir(_activeJointNum);
        dReal distToRand = nearestNode->dist(randNode);
        dir = (randNode->getNodeConfig() - nearestNode->getNodeConfig()) * (_stepSize / distToRand);

        vector<dReal> newConfig(_activeJointNum);
        newConfig = nearestNode->getNodeConfig();
        RRTNode* extendNode = new RRTNode(newConfig, nearestNode);  // its the nearestNode
        RRTNode* newNode = nullptr;

        while (true) {
            if (goalCheck(newConfig, randNode)){
                randNode->setParent(extendNode);
                _tree.addNode(randNode);
                break;
            }

            newConfig = extendNode->getNodeConfig() + dir;
            if (checkColl(newConfig)){
                break;
            }

            newNode = new RRTNode(newConfig, extendNode);
            _tree.addNode(newNode);
            extendNode = newNode;
        }

        return newNode;
    }

    void excuteTrajFromPathConfigs(vector<vector<dReal>>& pathConfigs)
    {
        if(pathConfigs.size()){
            cout << "##################" <<endl;
            TrajectoryBasePtr traj = RaveCreateTrajectory(_env, "");	
            traj->Init(_robot->GetActiveConfigurationSpecification());
            for (int i = 0; i < pathConfigs.size(); i++){
                traj->Insert(i, pathConfigs[i]);
            }
            planningutils::RetimeActiveDOFTrajectory(traj,_robot);
            _robot->GetController()->SetPath(traj);
        }
    }

    void drawPath(vector<vector<dReal>>& pathConfigs)
    {
        // draw elbow instead of eof
        // float red[4]={1,0,0,1};
        // // float blue[4]={0,0,1,1};
        // RobotBase::ManipulatorPtr manip = nullptr;
        // Transform T;
        // if(pathConfigs.size()){
        //     for (int i = 0; i < pathConfigs.size(); i++){
        //         _robot->SetActiveDOFValues(pathConfigs[i]);
        //         manip = _robot->GetActiveManipulator();
        //         T = manip->GetEndEffectorTransform();
        //         vector<float> points;
        //         points.push_back((float)T.trans.x);
        //         points.push_back((float)T.trans.y);
        //         points.push_back((float)T.trans.z);
        //         points.push_back(1);
        //         handles.push_back(_env->plot3(&points[0],1,1,5,red,0));
        //     }
            
        // }
        std::vector<double> links;
        std::vector<float> points;
        float red[4]={1,0,0,1};
        for(size_t i = 0; i < pathConfigs.size();i++) {
            points.clear();
            links.clear();
            for(size_t j = 0; j < pathConfigs[i].size();j++)
                links.push_back(pathConfigs[i][j]);
            _robot->SetActiveDOFValues(links);
            Transform T = _robot->GetLinks()[50]->GetTransform();
            points.push_back((float)T.trans.x);
            points.push_back((float)T.trans.y);
            points.push_back((float)T.trans.z);
            points.push_back(1);
            handles.push_back(_env->plot3(&points[0],1,1,5,red,0));
        }  
    }

    void drawPathBlue(vector<vector<dReal>>& pathConfigs)
    {
        // float red[4]={1,0,0,1};
        float blue[4]={0,0,1,1};
        // RobotBase::ManipulatorPtr manip = nullptr;
        // Transform T;
        // if(pathConfigs.size()){
        //     for (int i = 0; i < pathConfigs.size(); i++){
        //         _robot->SetActiveDOFValues(pathConfigs[i]);
        //         manip = _robot->GetActiveManipulator();
        //         T = manip->GetEndEffectorTransform();
        //         vector<float> points;
        //         points.push_back((float)T.trans.x);
        //         points.push_back((float)T.trans.y);
        //         points.push_back((float)T.trans.z);
        //         points.push_back(1);
        //         handles.push_back(_env->plot3(&points[0],1,1,5,blue,0));
        //     }
            
        // }
        std::vector<double> links;
        std::vector<float> points;
        for(size_t i = 0; i < pathConfigs.size();i++) {
            points.clear();
            links.clear();
            for(size_t j = 0; j < pathConfigs[i].size();j++)
                links.push_back(pathConfigs[i][j]);
            _robot->SetActiveDOFValues(links);
            Transform T = _robot->GetLinks()[50]->GetTransform();
            points.push_back((float)T.trans.x);
            points.push_back((float)T.trans.y);
            points.push_back((float)T.trans.z);
            points.push_back(1);
            handles.push_back(_env->plot3(&points[0],1,1,5,blue,0));
        }
    }

    vector<vector<dReal>> smoothPath(vector<vector<dReal>>& pathConfigs) {
        vector<vector<dReal>> smoothedPath = pathConfigs;
        int currIter = 0;
        int idx1, idx2, tmp;
        vector<dReal> smoothedPathLength;
        smoothedPathLength.push_back(calPathLength(smoothedPath));
        while (currIter < _smoothMaxIter){
            currIter++;
            idx1 = rand()%(smoothedPath.size());
            idx2 = rand()%(smoothedPath.size());

            if (idx1 == idx2){
                smoothedPathLength.push_back(calPathLength(smoothedPath));
                continue;
            }

            if (idx2 < idx1){ // make sure idx1 < idx2
                tmp = idx1;
                idx1 = idx2;
                idx2 = tmp;
            }

            vector<dReal> dir(_activeJointNum);
            dReal distToRand = 0.0;
            for (int i = 0; i < _activeJointNum; i++){
                distToRand += pow(smoothedPath[idx2][i] - smoothedPath[idx1][i], 2);
            }
            distToRand = sqrt(distToRand);
            dir = (smoothedPath[idx2] - smoothedPath[idx1]) * (_stepSize / distToRand);

            if (distToRand <= _stepSize){
                smoothedPathLength.push_back(calPathLength(smoothedPath));
                continue;
            }

            bool collFlag = false;
            int j = 0;
            vector<dReal> newConfig(_activeJointNum);
            vector<dReal> sStartConfig = smoothedPath[idx1];
            vector<vector<dReal>> exchangeConfigs;
            while (j < (int)(distToRand/_stepSize)){
                j++;
                newConfig = sStartConfig + dir;
                if (checkColl(newConfig)){
                    collFlag = true;
                    break;
                }
                exchangeConfigs.push_back(newConfig);
                sStartConfig = newConfig;
            }

            if (!collFlag){
                smoothedPath.erase(smoothedPath.begin()+idx1+1, smoothedPath.begin()+idx2);
                smoothedPath.insert(smoothedPath.begin()+idx1+1, exchangeConfigs.begin(), exchangeConfigs.end());
            }
            exchangeConfigs.clear();
            smoothedPathLength.push_back(calPathLength(smoothedPath));
        }
        // for (auto it = smoothedPathLength.begin(); it != smoothedPathLength.end(); it++)
        //     std::cout << *it << ", ";
        // std::cout << std::endl;
        return smoothedPath;
    }

    dReal distBwtConfigs(vector<dReal>& c1, vector<dReal>& c2){
        vector<dReal> disCon = c1-c2;
        dReal dis = 0.0;
        for (int i = 0; i < c1.size(); i++){
            dis += disCon[i] * disCon[i];
        }
        return sqrt(dis);
    }
    dReal calPathLength(vector<vector<dReal>>& pathConfigs){
        dReal pathDist = 0.0;
        for (int i = 1; i < pathConfigs.size(); i++){
            pathDist += distBwtConfigs(pathConfigs[i], pathConfigs[i-1]);
        }
    }

    bool getRRTPath(std::ostream& sout, std::istream& sinput)
    {
        // cout << "plannnnnnnnnnnnnning" <<endl;
        clock_t start, end;
        start = clock();
        RRTNode* startNode = new RRTNode(_startConfig);
        _tree.addNode(startNode);
        RRTNode* goalNode = new RRTNode(_goalConfig);
        // _tree.addNode(goalNode);
        int currIter = 0;
        vector<vector<dReal>> finalPathConfigs;
        while (currIter < _maxIter) {
            // generate a random node
            RRTNode* randNode = new RRTNode(randomGenCon());
            // for (auto it = randNode->getNodeConfig().begin(); it != randNode->getNodeConfig().end(); it++)
            //     std::cout << *it << " ";
            // std::cout << std::endl;
            // cout << "success generate randNode" <<endl;

            // expand the tree
            RRTNode* reachedNode = expandTree(randNode);
            // cout << "success expand tree" <<endl;
            
            // for (auto it = reachedNode->getNodeConfig().begin(); it != reachedNode->getNodeConfig().end(); it++)
            //     std::cout << *it << " ";
            // std::cout << std::endl;
            // cout << reachedNode->getNodeConfig()<<endl;

            // check if we reach the goal position
            if (reachedNode != nullptr && goalCheck(_goalConfig, reachedNode)){

                goalNode->setParent(reachedNode);
                _tree.addNode(goalNode);
                finalPathConfigs = _tree.getPath(_tree.getNTreeSize()-1);
                end = clock();
                cout << "The computation time for the RRT: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds" << endl;
                // excuteTrajFromPathConfigs(finalPathConfigs);
                
                drawPath(finalPathConfigs);

                // smooth path and print it
                vector<vector<dReal>> smoothedPath;
                start = clock();
                smoothedPath = smoothPath(finalPathConfigs);
                end = clock();
                cout << "The computation time for smoothing: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds" << endl;
                drawPathBlue(smoothedPath);
                excuteTrajFromPathConfigs(smoothedPath);
                cout << "The number of nodes sampled:" << _tree.getNTreeSize() << endl;
                cout << "The length of path(unsmoothed): " << finalPathConfigs.size() << endl;
                cout << "The length of path(smoothed): " << smoothedPath.size() << endl;
                cout << "The length of path(unsmoothed) in euclidean distance: " << calPathLength(finalPathConfigs)  << endl;
                cout << "The length of path(smoothed) in euclidean distance: " << calPathLength(smoothedPath) << endl;
                return true;
            }
            // cout << "success here" <<endl;
            currIter++;
            // cout << "iteration times: " << currIter << endl;
        }
        // cout << "success end" <<endl;
        return false;
    }

private:
    EnvironmentBasePtr _env;
    RobotBasePtr _robot;
    vector<dReal> _lowerLimits;
    vector<dReal> _upperLimits;
    int _activeJointNum;

    NodeTree _tree;
    vector<dReal> _startConfig;
    vector<dReal> _goalConfig;
    dReal _goalBias;
    dReal _stepSize = 0.2;
    int _maxIter = 500000;
    int _smoothMaxIter = 200;
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtmodule" ) {
        return InterfaceBasePtr(new RRTModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("RRTModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
