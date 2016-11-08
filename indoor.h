#include "tinyxml2.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <Winsock.h>
#ifndef XMLCheckResult
	#define XMLCheckResult(a_eResult) if (a_eResult != XML_SUCCESS) { printf("Error: %i\n", a_eResult); return a_eResult; }
#endif

using namespace std;
using namespace cv;
using namespace tinyxml2;

class Node {
private:
    string stateID;
    string gmlID;
    int posX;
    int posY;
    int posZ;
public:
    Node(){
       stateID;
       gmlID;
       posX = 0;
       posY = 0;
       posZ = 0;
    }

    Node(string stateId, string gmlId, int pos1, int pos2, int pos3){
        stateID = stateId;
        gmlID = gmlId;
        posX = pos1;
        posY = pos2;
        posZ = pos3;
    }

    void print_Vertex(){
        cout << "stateID : " << this->stateID << endl;
        cout << "gmlID : " << this->gmlID << endl;
        cout << "posX : " << this->posX << ", posY : " << this->posY << ", posZ : " << this->posZ << endl;
    }

    string get_stateID(){
        return this->stateID;
    }

    int get_posX() {
        return this->posX;
    }
    int get_posY() {
        return this->posY;
    }
    int get_posZ() {
        return this->posZ;
    }
};
bool nodeY_CMP(Node a, Node b) {
    return a.get_posY() < b.get_posY();
}
bool nodeX_CMP(Node a, Node b){
    if (a.get_posX() == b.get_posX()) {
        return nodeY_CMP(a, b);
    }
    return a.get_posX() < b.get_posX();
}

bool node_CMP(Node a, Node b)
{
    if (a.get_posZ() == b.get_posZ()) {
        return nodeX_CMP(a, b);
    }
    else return a.get_posZ() < b.get_posZ();

}
class Edge {
private:
        string transitionID;
        string connect1;
        string connect2;
        string gmlID;
        int* spos;
        int* dpos;

public:
    Edge()
    {
        transitionID;
        connect1;
        connect2;
        gmlID;
        spos = new int[3];
        dpos = new int[3];
    }
    Edge(string transitionId, string gmlId, string con1, string con2, int *src, int* dst ){
        transitionID = transitionId;
        connect1 = con1.erase(0,1);
        connect2 = con2.erase(0,1);
        gmlID = gmlId;
        spos = new int[3];
        dpos = new int[3];

        for(int i=0; i< 3; i++)
        {
            spos[i] = src[i];
            dpos[i] = dst[i];
        }
    }

    void print_Edge(){
        cout << "transitionID : " << this->transitionID << endl;
        cout << "connect1 : " << this->connect1 << endl;
        cout << "connect2 : " << this->connect2 << endl;
        cout << "gmlID : " << this->gmlID << endl;

        cout << "sposX : " << this->spos[0] << ", sposY : " << this->spos[1] << ", sposZ : " << this->spos[2] << endl;
    }

    string get_connect1(){
        return this->connect1;
    }
    string get_connect2(){
        return this->connect2;
    }

    int* get_spos() {
        return spos;
    }

    int* get_dpos() {
        return dpos;
    }
};

const string stateMember = "stateMember";
const string transitionMember = "transitionMember";

const int port = 4000;
const int thresh = 50; //40
const int max_thresh = 170;
//const string PATH = "C:/Users/Administrator/Desktop/node/photo/";
const string PATH = "C:/Users/Administrator/Desktop/drone/nodejs/photo/";
const string NodejsPath = PATH + "../"; //Change 6th Param to your downloaded node.js file path
const string SampleImagePath = PATH + "../../sample/";

fstream outfile;
SOCKET mySocket;
vector <Node> indoor_Node;
vector <Edge> indoor_Edge;
map<string, int> nodeList;
map<string, int>::iterator nodeIter;
vector<int> path;
double dist[101][101];
int djkstra[101];
int visit[101];
int route[101];
Mat GRAY_IMG;
Mat EDGE;
Mat COLOR_IMG;
Mat number[10];
int frameNumber = 1;
string IMG;
vector<double> value;


void sockSetAddress(struct sockaddr_in * pSockAddr, char *pstrIP, unsigned short nPort);
SOCKET ServerRunning();
void RunNodeJS();
vector<Point> Contours();
static double angle(Point pt1, Point pt2, Point pt0);
bool ImageSelect();
vector<Point> dot_sort(vector<Point> square_dot);
bool comp (Point i, Point j) { return (i.y<j.y); }
void ReadTemplateNumber();
Mat thresholding(Mat);
Mat Dilation(Mat);
int getNumber(Mat, Mat*, Mat*);
int Detecting(Mat input);
Mat Transformation(Mat, vector<Point>);
vector<int> FindPath();
vector<int> dijkstra(const int start, const int dest, const int nodeNum);
void readyDijkstra();
int read_IndoorXML();
double getDistance(int *spos, int*dpos);
Edge read_Edge(XMLNode* input);
Node read_Node(XMLNode* input);
void sockSetAddress(struct sockaddr_in * pSockAddr, char *pstrIP, unsigned short nPort);
SOCKET ServerRunning();
void RunNodeJS();
int show_direct(vector<int> path, int src, int angleFlag);
bool ImageSelect(int);
