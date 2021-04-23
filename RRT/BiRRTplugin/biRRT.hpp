#ifndef RRT_HPP
#define RRT_HPP
# include <vector>
using namespace OpenRAVE;
using namespace std;
int jointNum = 7;

class RRTNode 
{   private:
        vector<dReal> _q;
        RRTNode* _parent;
    public:
        RRTNode(vector<dReal> config)
        :_q(config)
        ,_parent(nullptr)
        {    
        }

        RRTNode(vector<dReal> config, RRTNode* parent)
        :_q(config)
        ,_parent(parent)
        {
        }


        void setParent(RRTNode* parent)
        {
            _parent = parent;
        }

        vector<dReal> getNodeConfig()
        {
            return _q;
        }

        RRTNode* getParent()
        {
            return _parent;
        }

        dReal dist(RRTNode* node2)
        {
            vector<dReal> q2 = node2->getNodeConfig();
            dReal distNN = 0.0;
            for (int i = 0; i < _q.size(); i++) {
                distNN += pow(_q[i] - q2[i], 2);
            }
            return sqrt(distNN);
        }
};

class NodeTree 
{
    private:
        vector<RRTNode*> _nodes;
    public:
        NodeTree()
        {
        _nodes.clear();
        }
        
        NodeTree(vector<RRTNode*> inputNodes)
        {
            _nodes = inputNodes;
        }

        void addNode(RRTNode* newNode)
        {
            _nodes.push_back(newNode);
        }

        void deleteNode(int idx)
        {
            _nodes.erase(_nodes.begin() + idx);
        }

        RRTNode* getNode(int idx)
        {
            return _nodes[idx];
        }

        vector<vector<dReal>> getPath(int idx, bool needrev)
        {
            RRTNode* curr = _nodes[idx];
            vector<vector<dReal>> path;
            int i = 0;
            while (curr != nullptr){// || i< 1E5){
                i++;
                // cout << i << endl;
                vector<dReal> currConfig = curr->getNodeConfig();
                path.push_back(currConfig);
                curr = curr->getParent();
            }
            if (needrev){
                reverse(path.begin(), path.end());
            }
            return path;
        }

        int getNTreeSize()
        {
            return _nodes.size();
        }

        RRTNode* findnearestNei(RRTNode* node)
        {
            dReal closestDist = 1E6;
            RRTNode* nearestNode = nullptr;
            dReal currDist = 0.0;
            int startIdx = getNTreeSize();
            if (startIdx < 800){
                startIdx = 0;
            }
            else {
                startIdx -= 800;
            }
            for (int i = startIdx; i < getNTreeSize(); i++){
                currDist = node->dist(getNode(i));
                if (currDist < closestDist) {
                    closestDist = currDist;
                    nearestNode = getNode(i);
                }
            }
            return nearestNode;
        }

        ~NodeTree() {
            for (auto n : _nodes){
                delete n;
            }  
            _nodes.clear();
        }
};

#endif // RRT_HPP