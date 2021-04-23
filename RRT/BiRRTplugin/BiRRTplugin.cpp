#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <openrave/openrave.h>
#include <openrave/planningutils.h>
#include <math.h>
#include <time.h>
#include <vector>
#include <random>
#include "biRRT.hpp"
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

class BiRRTModule : public ModuleBase
{
public:
    BiRRTModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&BiRRTModule::MyCommand,this,_1,_2),
                        "This is an example command");

        RegisterCommand("passEnvRobot",boost::bind(&BiRRTModule::passEnvRobot,this,_1,_2),
                        "This is to pass env into the cpp here");
        RegisterCommand("setStartConfig",boost::bind(&BiRRTModule::setStartConfig,this,_1,_2),
                        "This is to set startConfig");
        RegisterCommand("setGoalConfig",boost::bind(&BiRRTModule::setGoalConfig,this,_1,_2),
                        "This is to pass goal to the RRT");
        RegisterCommand("setGoalBias",boost::bind(&BiRRTModule::setGoalBias,this,_1,_2),
                        "This is to set goal bias variable");
        RegisterCommand("getRRTPath",boost::bind(&BiRRTModule::getRRTPath,this,_1,_2),
                        "This is to excute RRT to get the path");

    }
    virtual ~BiRRTModule() {}
    
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
        RRTNode* nearestNode = nullptr;
        // if (!goalSide) {
            nearestNode = _tree.findnearestNei(randNode);
        // } else {
        //     nearestNode = _golaDIRtree.findnearestNei(randNode);
        // }
        // cout << "inexpand, nearestNode" <<endl;
        // calculate the direction for expand
        vector<dReal> dir(_activeJointNum);
        // cout << "inexpand, cal distToRand " <<endl;
        dReal distToRand = nearestNode->dist(randNode);
        if (distToRand < _stepSize){
            return nullptr;
        }
        // cout << "inexpand, distToRand: " << distToRand <<endl;
        dir = (randNode->getNodeConfig() - nearestNode->getNodeConfig()) * (_stepSize / distToRand);

        
        vector<dReal> newConfig(_activeJointNum);
        newConfig = nearestNode->getNodeConfig();
        RRTNode* extendNode = new RRTNode(newConfig, nearestNode);  // its the nearestNode
        RRTNode* newNode = nullptr;

        // cout << "inexpand, before while" << endl;
        int extendIter = 0;
        while (true) {
            extendIter++;
            if (goalCheck(newConfig, randNode)){
                // cout << "WWW 111111" << endl;
                randNode->setParent(extendNode);
                // if (!goalSide) {
                    _tree.addNode(randNode);
                // } else {
                //     _golaDIRtree.addNode(randNode);
                // }

                break;
            }
            // cout << "WWW 2222" << endl;
            newConfig = extendNode->getNodeConfig() + dir;
            if (checkColl(newConfig)){
                break;
            }
            // cout << "WWW 3333" << endl;
            newNode = new RRTNode(newConfig, extendNode);
            // if (!goalSide) {
                _tree.addNode(newNode);
            // } else {
            //     _golaDIRtree.addNode(newNode);
            // }
            extendNode = newNode;
        }
        dir.clear();
        newConfig.clear();
        return newNode;
    }

    RRTNode* expandAnTree(RRTNode* target)
    {   
        // find the nearestNode
        // for (auto it = target->getNodeConfig().begin(); it != target->getNodeConfig().end(); it++)
        //         std::cout << *it << " ";
        //     std::cout << std::endl;
        RRTNode*  nearestNode = _golaDIRtree.findnearestNei(target);
        // cout << "inexpand, nearestNode" <<endl;
        // calculate the direction for expand
        vector<dReal> dir(_activeJointNum);
        // cout << "inexpand, cal distToRand " <<endl;
        dReal distToRand = nearestNode->dist(target);
        // cout << "inexpand, distToRand: " << distToRand <<endl;
        dir = (target->getNodeConfig() - nearestNode->getNodeConfig()) * (_stepSize / distToRand);

        
        vector<dReal> newConfig(_activeJointNum);
        newConfig = nearestNode->getNodeConfig();
        RRTNode* extendNode = new RRTNode(newConfig, nearestNode);  // its the nearestNode
        RRTNode* newNode = nullptr;

        // cout << "inexpand, before while" << endl;
        while (true) {
            if (goalCheck(newConfig, target)){
                // cout << "WWW 111111" << endl;
                target->setParent(extendNode);
                _golaDIRtree.addNode(target);

                break;
            }
            // cout << "WWW 2222" << endl;
            newConfig = extendNode->getNodeConfig() + dir;
            if (checkColl(newConfig)){
                break;
            }
            // cout << "WWW 3333" << endl;
            newNode = new RRTNode(newConfig, extendNode);
            _golaDIRtree.addNode(newNode);

            extendNode = newNode;
        }
        dir.clear();
        newConfig.clear();
        return newNode;
    }

    void excuteTrajFromPathConfigs(vector<vector<dReal>>& pathConfigs)
    {
        if(pathConfigs.size()){
            // cout << "##################" <<endl;
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

        while (currIter < _smoothMaxIter){
            currIter++;
            idx1 = rand()%(smoothedPath.size()-1);
            idx2 = rand()%(smoothedPath.size()-1);

            if (idx1 == idx2){
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
        }
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
        _golaDIRtree.addNode(goalNode);
        int currIter = 0;
        bool goalDIRTimes = true;
        
        vector<vector<dReal>> finalPathConfigsStartSide;
        vector<vector<dReal>> finalPathConfigsGoalSide;
        vector<vector<dReal>> finalPathConfigs;
        // cout << "here" <<endl;
        while (currIter < _maxIter) {
            // generate a random node
            
            RRTNode* randNode = new RRTNode(randomGenCon());
            // for (auto it = randNode->getNodeConfig().begin(); it != randNode->getNodeConfig().end(); it++)
            //     std::cout << *it << " ";
            // std::cout << std::endl;
            // cout << "success generate randNode-----------------" <<endl;

            // expand the tree
            RRTNode* reachedNode = expandTree(randNode); 
            // cout << "success expand tree1-----------" <<endl;
            // for (auto it = reachedNode->getNodeConfig().begin(); it != reachedNode->getNodeConfig().end(); it++)
            //     std::cout << *it << " ";
            // std::cout << std::endl;
            // cout << "---------" <<endl;
            // RRTNode* tarNode = new RRTNode(reachedNode->getNodeConfig());
            // RRTNode* reachedNodeGoalSide = nullptr;
            // reachedNodeGoalSide = expandAnTree(randNode);
            RRTNode* reachedNodeGoalSide = nullptr;
            if (reachedNode != nullptr) {
                RRTNode* tarNode = new RRTNode(reachedNode->getNodeConfig());
                reachedNodeGoalSide = expandAnTree(tarNode);
                // reachedNodeGoalSide = expandAnTree(reachedNode);
                // cout << "success expand tree2" <<endl;
            }
            
            // delete randNode;
            // for (auto it = reachedNode->getNodeConfig().begin(); it != reachedNode->getNodeConfig().end(); it++)
            //     std::cout << *it << " ";
            // std::cout << std::endl;
            // cout << reachedNode->getNodeConfig()<<endl;

            // check if we reach the goal position
            if (reachedNode != nullptr && reachedNodeGoalSide != nullptr && goalCheck(reachedNodeGoalSide->getNodeConfig(), reachedNode)){
            // if (reachedNode != nullptr && goalCheck(_goalConfig, reachedNode)){
                // cout << "In path Nowwww%%%%%%%%%%%" <<endl;
                // cout <<_tree.getNTreeSize() << endl;

                finalPathConfigsGoalSide = _golaDIRtree.getPath(_golaDIRtree.getNTreeSize()-1, false);
                // finalPathConfigsGoalSide = _golaDIRtree.getPath(0, true);
                // cout << "2222222222%%%%%%%%%%%" <<endl;

                reachedNodeGoalSide->setParent(reachedNode);
                _tree.addNode(reachedNodeGoalSide);
                // goalNode->setParent(reachedNode);
                // _tree.addNode(goalNode);
                // cout <<_tree.getNTreeSize() << endl;
                finalPathConfigsStartSide = _tree.getPath(_tree.getNTreeSize()-1, true);
                // cout << "1111111111%%%%%%%%%%%" <<endl;
                // reachedNode->setParent(reachedNodeGoalSide);
                // _golaDIRtree.addNode(reachedNode);
                // finalPathConfigsGoalSide = _golaDIRtree.getPath(_golaDIRtree.getNTreeSize()-1, false);
                // // finalPathConfigsGoalSide = _golaDIRtree.getPath(0, true);
                // cout << "2222222222%%%%%%%%%%%" <<endl;
                
                    
                // cout << finalPathConfigsStartSide.size() <<endl;
                // cout << finalPathConfigsGoalSide.size() <<endl;
                // finalPathConfigs = finalPathConfigsStartSide;
                finalPathConfigs.insert(finalPathConfigs.end(), finalPathConfigsStartSide.begin(), finalPathConfigsStartSide.end());
                finalPathConfigs.insert(finalPathConfigs.end(), finalPathConfigsGoalSide.begin(), finalPathConfigsGoalSide.end());
                // cout << finalPathConfigs.size() <<endl;
                end = clock();

                cout << "The computation time for the biRRT: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds" << endl;
                // cout <<  (double)(end - start) / CLOCKS_PER_SEC  << endl;
                // excuteTrajFromPathConfigs(finalPathConfigs);
                
                drawPath(finalPathConfigs);

                // smooth path and print it
                vector<vector<dReal>> smoothedPath;
                start = clock();
                smoothedPath = smoothPath(finalPathConfigs);
                end = clock();

                cout << "The computation time for smoothing: " << (double)(end - start) / CLOCKS_PER_SEC << " seconds" << endl;
                // cout << (double)(end - start) / CLOCKS_PER_SEC << endl;
                drawPathBlue(smoothedPath);
                excuteTrajFromPathConfigs(smoothedPath);
                // cout << _tree.getNTreeSize() << endl;
                cout << "The number of nodes sampled:" << _tree.getNTreeSize() << endl;
                cout << "The length of path(unsmoothed) in vector size: " << finalPathConfigs.size() << endl;
                cout << "The length of path(smoothed) in vector size: " << smoothedPath.size() << endl;
                cout << "The length of path(unsmoothed) in euclidean distance: " << calPathLength(finalPathConfigs)  << endl;
                cout << "The length of path(smoothed) in euclidean distance: " << calPathLength(smoothedPath) << endl;
                // cout <<  calPathLength(finalPathConfigs)  << endl;
                return true;
            }
            // cout << "success here" <<endl;
            currIter++;
            // cout << "iteration times: " << currIter << endl;
        }
        // cout << "success end" <<endl;
        // delete startNode;
        // delete goalNode;
        return false;
    }

private:
    EnvironmentBasePtr _env;
    RobotBasePtr _robot;
    vector<dReal> _lowerLimits;
    vector<dReal> _upperLimits;
    int _activeJointNum;

    NodeTree _tree;
    NodeTree _golaDIRtree;
    vector<dReal> _startConfig;
    vector<dReal> _goalConfig;
    dReal _goalBias;
    dReal _stepSize = 0.2;
    int _maxIter = 10000;
    int _smoothMaxIter = 200;
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "birrtmodule" ) {
        return InterfaceBasePtr(new BiRRTModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("BiRRTModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

