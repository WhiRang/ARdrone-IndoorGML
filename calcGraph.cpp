///************************************************************************************///
///                          2016 Winter Graduation assignments                        ///
///                          Team Overload                                             ///
///                          Professor : Prof.Ki-Joune Li                              ///
///                          Subject : Indoor driving drone                            ///
///                          Member : KSW, RWY, HSH                                    ///
///************************************************************************************///

#include "indoor.h"
using namespace tinyxml2;
using namespace cv;
using namespace std;

int main() {
    outfile.open("./log.txt", ios::out);
    path = FindPath();
    int node;
    int recv_ret;
    char recvbuf[4096];
    int recvbuflen = 4096;
    string currentimg;
    ReadTemplateNumber();
    mySocket = ServerRunning();

    string cmd = "6";
    cout << "take off" << endl;
    outfile << "take off" << endl;
    Sleep(500);
    cmd = "7";
    send(mySocket, cmd.c_str(), cmd.size(),0);
    Sleep(4000);

    cout << "drone up" << endl;
    outfile << "drone up" << endl;
    cmd = "8";
    send(mySocket, cmd.c_str(), cmd.size(),0);
    Sleep(3500);

    cout << "drone stop" << endl;
    outfile << "drone stop" << endl;
    cmd = "7";
    send(mySocket, cmd.c_str(), cmd.size(),0);
    Sleep(100);

    do {
        recv_ret = recv(mySocket, recvbuf, recvbuflen, 0);
        recvbuf[recv_ret] = NULL;
        char *ptr;
        char *pptr = NULL;
        ptr = strtok(recvbuf, "g");
        while(pptr = strtok(NULL,"g")) {
            ptr = pptr;
        }
        currentimg = ptr;
        currentimg += "g";
        cout << "current img : " << currentimg << endl;
        outfile << "current img : " << currentimg <<endl;
        IMG = PATH + currentimg;
        GRAY_IMG = imread(IMG, CV_LOAD_IMAGE_GRAYSCALE);
        EDGE = Mat(GRAY_IMG.size(), CV_8UC1);
        Canny(GRAY_IMG, EDGE, thresh, max_thresh, 3);

        vector<Point> dots = Contours();
        int angle = 0;
        /** return Data
           Error
           -666 : end
           Except
            -1  : stack's data != input
           Correct
            1 : 90 // right
            2 : 180 // front
            3 : 270 // left
            6 : land
            7 : 360 // back
        **/
        if (dots.size() == 4) {
            node = Detecting(Transformation(GRAY_IMG, dots));
            int command = show_direct(path, node/10, 2);
            if (command == -1 || command == 2) {
                cmd = "2";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                outfile << "go & command : 2" << endl;
                Sleep(200);
            }
            if (command == 1) {
                angle += 1; // turn right
                cmd = "2";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(500);
                cmd = "7";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(100);
                cmd = "1";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(1400);
                cmd = "7";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(100);
                cmd = "2";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(500);
                outfile << "turn right & command : 1" <<endl;
            }
            if (command == 3) {
                angle += 3; // turn left
                cmd = "2";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(500);
                cmd = "7";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(100);
                cmd = "3";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(1400);
                cmd = "7";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(100);
                cmd = "2";
                send(mySocket, cmd.c_str(), cmd.size(),0);
                Sleep(500);
                outfile << "turn right & command : 3" <<endl;
            }
            if (command == 6) {
                Sleep(500);
                cout << "Land" << endl;
                cmd = "6";
                path.pop_back();
                send(mySocket, cmd.c_str(), cmd.size(),0);
                outfile << "Land & command : 6" <<endl;
                break;
            }
            if (command > 0) {
                path.pop_back();
            }
        }
        else {
            cout << "cannot find marker" << endl;
            outfile << "cannot find marker" << endl;
        }
    } while (recv_ret > 0);
    outfile.close();
    cmd = "6";
    send(mySocket, cmd.c_str(), cmd.size(),0);
    printf("SUCCESS\n");
    closesocket(mySocket);
    WSACleanup();
    return 0;
}

vector<Point> Contours() {
    bool flag = false;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Point> approx;
    vector<Point> output;
    vector<vector<Point> > contours_poly;
    contours_poly.clear();
    output.clear();
    findContours(EDGE, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    for (unsigned int i = 0; i < contours.size(); i++) {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
        if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && fabs(contourArea(Mat(approx))) < 50000 && isContourConvex(Mat(approx))) {
            double maxCosine = 0;
            for (int j = 2; j < 5; j++) {
                double cosine = fabs(angle(approx[j % 4], approx[j - 2], approx[j - 1]));
                maxCosine = MAX(maxCosine, cosine);
            }
            if (maxCosine < 0.3) {
                flag = true;
                approx = dot_sort(approx);
                output = approx;
                contours_poly.push_back(approx);
            }
        }
    }

    if (flag == true) {

        Point center = Point((contours_poly[contours_poly.size()-1][0].x + contours_poly[contours_poly.size()-1][3].x) / 2, (contours_poly[contours_poly.size()-1][0].y + contours_poly[contours_poly.size()-1][3].y) / 2);

        if (center.x > 370) { // >>>>치우쳐짐
            string sendbuf = "5";
            send(mySocket, sendbuf.c_str(), sendbuf.size(),0);
            Sleep(50);
            cout << "move right" << endl;
            outfile << "move right" <<endl;
        }
        else if (center.x < 270) { // <<<<< 치우쳐짐.
            string sendbuf = "4";
            send(mySocket, sendbuf.c_str(), sendbuf.size(),0);
            Sleep(50);
            cout << "move left" << endl;
            outfile << "move left" <<endl;
        }

    }
    return output;
}

static double angle(Point pt1, Point pt2, Point pt0) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

vector<Point> dot_sort(vector<Point> square_dot) { // 왼쪽 위, 오른 쪽 위, 왼쪽 아래, 오른쪽 아래 순으로 vector sort
    vector<Point> trans;
    sort(square_dot.end() - 4, square_dot.end(), comp); //사각형 점들의 집합 중 가장 작은 사각형 점 4개 (맨 마지막 4개)만 y 기준 sort 실행
    Point temp;
    // 위 점 2개의 x 기준 sort
    if (square_dot[square_dot.size() - 4].x > square_dot[square_dot.size() - 3].x) {

        temp = square_dot[square_dot.size() - 4];
        square_dot[square_dot.size() - 4] = square_dot[square_dot.size() - 3];
        square_dot[square_dot.size() - 3] = temp;
    }

   //아래 점 2개의 x 기준 sort
    if (square_dot[square_dot.size() - 2].x > square_dot[square_dot.size() - 1].x) {
        Point temp;
        temp = square_dot[square_dot.size() - 2];
        square_dot[square_dot.size() - 2] = square_dot[square_dot.size() - 1];
        square_dot[square_dot.size() - 1] = temp;
    }

    //정렬된 4개의 점을 push하고 반환한다.
    trans.push_back(square_dot[square_dot.size() - 4]);
    trans.push_back(square_dot[square_dot.size() - 3]);
    trans.push_back(square_dot[square_dot.size() - 2]);
    trans.push_back(square_dot[square_dot.size() - 1]);
    return trans;
}

Mat Transformation(Mat input, vector<Point> test) {
    Mat number;
    number.rows = 120;
    number.cols = 260;
    Point2f inputQuad[4]; // Input Quadilateral or Image plane coordinates
    Point2f outputQuad[4]; // Output Quadilateral or World plane coordinates
    Mat lambda(2, 4, CV_32FC1); // Lambda Matrix
    Mat output; //Input and Output Image;
    lambda = Mat::zeros(number.rows, number.cols, input.type()); // Set the lambda matrix the same type and size as input

    inputQuad[0] = Point2f(test[3].x, test[3].y);
    inputQuad[1] = Point2f(test[0].x, test[2].y);
    inputQuad[2] = Point2f(test[2].x, test[0].y);
    inputQuad[3] = Point2f(test[1].x, test[1].y);

    outputQuad[0] = Point2f(0, 0);
    outputQuad[1] = Point2f(number.cols - 1, 0);
    outputQuad[2] = Point2f(number.cols - 1, number.rows - 1);
    outputQuad[3] = Point2f(0, number.rows - 1);
    lambda = getPerspectiveTransform(inputQuad, outputQuad); // Get the Perspective Transform Matrix i.e. lambda
    warpPerspective(input, output, lambda, number.size()); // Apply the Perspective Transform just found to the src image

    return output;
}

int Detecting(Mat input) { // marker에서 관심영역(숫자)만 가지고 오기

    Mat dillate_first = Mat(input, cv::Rect(0, 10, 110, 90)).clone();
    Mat dillate_second = Mat(input, cv::Rect(80, 10, 110, 90)).clone();
    Mat dillate_third = Mat(input, cv::Rect(150, 10, 110, 90)).clone();

    Mat mark[3] = { dillate_first, dillate_second, dillate_third };

    return getNumber(input, mark, number);
}

Mat thresholding(Mat target) {
    int threshold_value = 95;//95;
    int threshold_type = 0;
    int const max_BINARY_value = 255;
    Mat dst;
    threshold(target, dst, threshold_value, max_BINARY_value, threshold_type);

    return dst;
}

Mat Dilation(Mat target) {
    Mat dilation_dst;
    int dilation_size = 2;
    Mat element = getStructuringElement(MORPH_ELLIPSE,
    Size(2 * dilation_size + 1, 2 * dilation_size + 1),
    Point(dilation_size, dilation_size));
    /// Apply the dilation operation
    dilate(target, dilation_dst, element);
    return dilation_dst;
}

int getNumber(Mat input, Mat* mark, Mat* number) {
    double minVal, maxVal;
    CvPoint left_top;
    IplImage *Ipl_marker, *Ipl_number;
    int answer[3] = { 0, 0, 0 };
    for (int i = 0; i<3; i++) {
        Ipl_marker = new IplImage(mark[i]);
        for (int j = 0; j < 10; j++) {
            Ipl_number = new IplImage(number[j]);
            IplImage* img = cvCreateImage(cvSize(Ipl_marker->width - Ipl_number->width + 1, Ipl_marker->height - Ipl_number->height + 1), IPL_DEPTH_32F, 1);
            cvMatchTemplate(Ipl_marker, Ipl_number, img, CV_TM_CCOEFF_NORMED);
            cvMinMaxLoc(img, &minVal, &maxVal, NULL, &left_top);
            value[j] += maxVal;
        }
        double findMax = value[0];
        int out = 0;
        for (int k=1; k<10; k++){
            if (findMax < value[k]) {
                findMax = value[k];
                out = k;
            }
        }
        for(int k=0; k<10; k++){
            value[k] = 0.0;
        }
        answer[i] = out;
    }

    cout << answer[0] << " " << answer[1] << " " << answer[2] << endl;
    outfile << answer[0] << " " << answer[1] << " " << answer[2] << endl;
    if (answer[0] < 0 || answer[1] < 0 || answer[2] < 0) {
        return -1;
    }
    return answer[0] * 100 + answer[1] * 10 + answer[2];
}

void ReadTemplateNumber() {
    number[0] = imread(SampleImagePath + "0.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[1] = imread(SampleImagePath + "1.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[2] = imread(SampleImagePath + "2.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[3] = imread(SampleImagePath + "3.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[4] = imread(SampleImagePath + "4.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[5] = imread(SampleImagePath + "5.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[6] = imread(SampleImagePath + "X.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[7] = imread(SampleImagePath + "7.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[8] = imread(SampleImagePath + "=.png", CV_LOAD_IMAGE_GRAYSCALE);
    number[9] = imread(SampleImagePath + "9.png", CV_LOAD_IMAGE_GRAYSCALE);
    for (int i = 0; i<10; i++) {
        value.push_back(0.0);
    }
}

Node read_Node(XMLNode* input) {
    XMLElement *nElement = input->ToElement();
    string stateID;
    string gmlID;
    string split;

    // stateMember -> State
    nElement = nElement->FirstChildElement();
    stateID = nElement->Attribute("gml:id");

    // state -> geometry -> gmlPoint
    nElement = nElement->FirstChildElement()->FirstChildElement();
    gmlID = nElement->Attribute("gml:id");

    // gmlPoint -> gmlPos
    nElement = nElement->FirstChildElement();
    string pos = nElement->GetText();
    stringstream split_stream(pos);

    double position[3];
    split_stream >> split;
    position[0] = (int)((atof(split.c_str()) - 129.082445858) * 100000000);

    split_stream >> split;
    position[1] = (int)((atof(split.c_str()) - 35.2348658873765) * 100000000);

    split_stream >> split;
    position[2] = (int)((atof(split.c_str())) * 10000);
    Node* tmp = new Node(stateID, gmlID, position[0], position[1], position[2]);

    return *tmp;
}

Edge read_Edge(XMLNode* input) {

    XMLElement * eElement = input->ToElement();
    string transitionID;

    eElement = eElement->FirstChildElement();
    transitionID = eElement->Attribute("gml:id");


    string connects1;
    string connects2;
    eElement = eElement->FirstChildElement();
    connects1 = eElement->Attribute("xlink:href");


    eElement = eElement->NextSiblingElement();
    connects2 = eElement->Attribute("xlink:href");

    eElement = eElement->NextSiblingElement();

    string gmlID;
    eElement = eElement->FirstChildElement();
    gmlID = eElement->Attribute("gml:id");


    eElement = eElement->FirstChildElement();
    string split;
    string pos;
    pos = eElement->GetText();
    stringstream split_stream(pos);

    int spos[3]; int dpos[3];

    split_stream >> split;
    spos[0] = (int)((atof(split.c_str()) - 129.082445858) * 100000000);

    split_stream >> split;
    spos[1] = (int)((atof(split.c_str()) - 35.2348658873765) * 100000000);

    split_stream >> split;
    spos[2] = (int)((atof(split.c_str())) * 10000);

    eElement = eElement->NextSiblingElement();
    pos = eElement->GetText();
    stringstream split_stream2(pos);

    split_stream2 >> split;
    dpos[0] = (int)((atof(split.c_str()) - 129.082445858) * 100000000);

    split_stream2 >> split;
    dpos[1] = (int)((atof(split.c_str()) - 35.2348658873765) * 100000000);


    split_stream2 >> split;
    dpos[2] = (int)((atof(split.c_str())) * 10000);


    Edge* tmp = new Edge(transitionID, gmlID, connects1, connects2, spos, dpos);

    return *tmp;
}

double getDistance(int *spos, int*dpos) {
    double length;

    int x = spos[0] - dpos[0];
    int y = spos[1] - dpos[1];
    int z = spos[2] - dpos[2];

    length = sqrt(x*x + y*y + z*z);

    return length;
}


int read_IndoorXML() {
    tinyxml2::XMLDocument doc;
    XMLError eResult = doc.LoadFile("PNU_313Building_3_4F.gml");
    XMLCheckResult(eResult);
    // MutiLayeredGraph
    XMLNode * pRoot = doc.FirstChild()->NextSibling()->NextSibling()->FirstChild();

    string str;
    int i = 0;

    while (pRoot != NULL) {

        // if pElement is stateMember's element.
        if (stateMember.compare(pRoot->Value()) == 0) {

            Node tmp = read_Node(pRoot);
            indoor_Node.push_back(tmp);

            if (pRoot->NextSibling() == NULL) {
            pRoot = pRoot->Parent()->NextSibling();
            }
            else {
            pRoot = pRoot->NextSibling();
            }
            i++;
        }

      // if pElement is transitionMember's element.
        else if (transitionMember.compare(pRoot->Value()) == 0) {
            indoor_Edge.push_back(read_Edge(pRoot));

            if (pRoot->NextSibling() == NULL) {
                break;
            }
            else {
                pRoot = pRoot->NextSibling();
            }
            i++;
        }
        else {
            pRoot = pRoot->FirstChild();
        }
    }

    sort(indoor_Node.begin(), indoor_Node.end(), node_CMP);

    for (unsigned int i = 0; i< indoor_Node.size(); i++) {
        nodeList.insert(pair<string, int>(indoor_Node[i].get_stateID(), i));
    }
}

vector<int> dijkstra(const int start, const int dest, const int nodeNum) {
    /** shortest path algorithm **/
    djkstra[start] = 0;
    int v;
    int min;

    for (int i = 1; i <= nodeNum; i++)
    {
        min = 2000000000;

        for (int j = 1; j <= nodeNum; j++) {
            if (visit[j] != 1) {
                if (min > djkstra[j]) {
                   v = j;
                   min = djkstra[j];
                }
            }
        }

        visit[v] = 1;

        for (int j = 1; j <= nodeNum; j++) {
            if (dist[v][j] != -1) {
                if (djkstra[j] > djkstra[v] + dist[v][j]) {
                    djkstra[j] = djkstra[v] + dist[v][j];
                    route[j] = v;
                }
            }
        }
    }

    /** print root. **/
    vector<int> path;
    int tmp = dest;
    path.push_back(tmp);
    do {
        path.push_back(route[tmp]);
        tmp = route[tmp];
    } while (route[tmp]);

    path.push_back(djkstra[dest]);
    return path;

}

void readyDijkstra() {

    int nodeNum = indoor_Node.size();
    // initialize dijkstra
    for (int i = 1; i <= nodeNum; i++) {
        for (int j = 1; j <= nodeNum; j++) {
            dist[i][j] = -1;
        }
        djkstra[i] = 2000000000;
        visit[i] = 0;
    }

   // compute dis & input
    for (unsigned int i = 0; i< indoor_Edge.size(); i++) {
        string src = indoor_Edge[i].get_connect1();
        string dst = indoor_Edge[i].get_connect2();
        int src_idx = (nodeIter = nodeList.find(src))->second;
        int dst_idx = (nodeIter = nodeList.find(dst))->second;
        src_idx += 1;
        dst_idx += 1;
        double tmp = getDistance(indoor_Edge[i].get_spos(), indoor_Edge[i].get_dpos());
        dist[src_idx][dst_idx] = tmp; dist[dst_idx][src_idx] = tmp;
        cout << (i + 1) << " from " << src_idx << " to " << dst_idx << " : " << tmp << endl;
    }

}

vector<int> FindPath() {

    read_IndoorXML();
    int nodeNum = indoor_Node.size();
    readyDijkstra();

    int start; int dest;

    for (int i = 0; i< nodeNum; i++) {
        int x; int y; int z;

        x = indoor_Node[i].get_posX();
        y = indoor_Node[i].get_posY();
        z = indoor_Node[i].get_posZ();
        nodeIter = nodeList.find(indoor_Node[i].get_stateID());
        int sector = nodeIter->second;
        sector += 1;
        cout << sector << " : " << x << "," << y << "," << z << endl;
    }

    cout << "Please input start node : " ;
    cin >> start;

    cout << "Please input destination node : ";
    cin >> dest;
    vector<int> path = dijkstra(start, dest, nodeNum);
    path.pop_back();

    return path;
}

void sockSetAddress(struct sockaddr_in * pSockAddr, char *pstrIP, unsigned short nPort)
{
    if (!pSockAddr) return;

    pSockAddr->sin_family = AF_INET;
    pSockAddr->sin_port = htons(nPort);

    if (pstrIP) {
        if (isdigit(pstrIP[0])) {
            pSockAddr->sin_addr.s_addr = inet_addr(pstrIP);
        }
        else {
            HOSTENT * pHostent;
            if (pHostent = gethostbyname(pstrIP))
            memcpy(&pSockAddr->sin_addr.s_addr, pHostent->h_addr, pHostent->h_length);
            else
            pSockAddr = NULL;
        }
    }
    else {
        pSockAddr->sin_addr.s_addr = htonl(INADDR_ANY);
    }
}
SOCKET ServerRunning() {
    WSADATA wsa;
    SOCKET socket_;
    SOCKET accept_sock;
    struct sockaddr_in   local_addr;
    struct sockaddr_in client_addr;
    int len_addr;

    //Winsock Initialization
    WSAStartup(MAKEWORD(2, 2), &wsa);

    //Open a Socket
    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    //Set local address & Bind
    sockSetAddress(&local_addr, NULL, port); // bind
    bind(socket_, (SOCKADDR *)&local_addr, sizeof(local_addr));

    // Listen
    listen(socket_, SOMAXCONN);
    printf("Server Running...\n");
    outfile << "Server Running..." << endl;
    RunNodeJS();

    // Accept, Receive & echo back
    len_addr = sizeof(client_addr);
    // Accept
    accept_sock = accept(socket_, (SOCKADDR *)&client_addr, &len_addr);

    inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port);
    return accept_sock;
}

void RunNodeJS() {
    ShellExecute(NULL, "open", "cmd.exe", "/C node newsocket.js", NodejsPath.c_str(), SW_NORMAL);
    cout << "execute Node.js" << endl;
    outfile << "execute Node.js" << endl;
}

double getAngle(int x1, int y1, int x3, int y3)
{
    #define PI 3.141592
    double a, b, c;
    double angle, temp;



    int n12x = x1;
    int n12y = y1;
    int n23x = x3;
    int n23y = y3;
    int n13x = x3-x1;
    int n13y = y3-y1;

    a = sqrt( pow( n13x, 2) + pow(n13y, 2));
    b = sqrt( pow( n12x, 2) + pow(n12y, 2));
    c = sqrt( pow( n23x, 2) + pow(n23y, 2));

    temp = ( pow(b,2) + pow(c,2) - pow(a,2)) / (2*b*c);

    angle = acos(temp);
    angle = angle * ( 180 / PI );

    return angle;
}

int show_direct(vector<int> path, int src, int angleFlag)
{
    int input = src-1;

    int now; int next;
    int v_Size = path.size();
    if(v_Size  == 0) return -666;
    now = path[v_Size-1] -1;
    if( now != input)   return -1;
    int angle;
    if(v_Size >= 2)
    {
        next = path[v_Size-2] -1;
        int x = indoor_Node[next].get_posX() - indoor_Node[now].get_posX();
        int y = indoor_Node[next].get_posY() - indoor_Node[now].get_posY();
/*
        for(int i=0; i< v_Size-1; i++) cout << path[i] << "\t";
        cout << endl;
*/
        int tmp;

        if( x > 0 && y > 0){
            angle = getAngle(x,0, x,y);
            if( angle < 45) tmp = 2;
            else tmp = 3;
        }
        else if (x > 0 && y <0){
            angle = getAngle(x,0, x,y);
            if( angle < 45) tmp = 2;
            else tmp = 1;
        }
        else if (x < 0 && y >0){
            angle = getAngle(x,0, x,y);
            if( angle < 45) tmp = 4; //
            else tmp = 3;
        }
        else{
            angle = getAngle(x,0, x,y);
            if( angle < 45) tmp = 4; //
            else tmp = 1;
        }
        tmp = (tmp + angleFlag)%4;
        if(tmp == 0) return 4;
        return tmp;
    }
    else
    {
        if(now == input){
            return 6;
        }
        else return -1;
    }

    return -1;
}
