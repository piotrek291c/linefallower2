
#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDebug>
#include<QUdpSocket>
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include<QtMath>
#include <QThread>
#include <QWebView>





#include <QWebHistory>
#include <QWebHistoryItem>
#include <QWebPage>
#include <QWebView>
#include <QtWebKit/QtWebKit>
#include <QtWebKitWidgets/QtWebKitWidgets> // With Qt >= 4.8
#include<QFile>
#include <QTextStream>
#include <cmath>
#include<QtMath>
#include<math.h>
#include <conio.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include<string>

using namespace std;

const int n=460; // horizontal size of the map
const int m=460; // vertical size size of the map
 int mapp[n][m];
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m]; // map of open (not-yet-tried) nodes
static int dir_map[n][m]; // map of directions
const int dir=8; // number of possible directions to go at any position
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dxx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dyy[dir]={0, 1, 1, 1, 0, -1, -1, -1};
class node
{
    // current position
    int xPos;
    int yPos;
    // total distance already travelled to reach the node
    int level;
    // priority=level+remaining distance estimate
    int priority;  // smaller: higher priority

    public:
        node(int xp, int yp, int d, int p)
            {xPos=xp; yPos=yp; level=d; priority=p;}

        int getxPos() const {return xPos;}
        int getyPos() const {return yPos;}
        int getLevel() const {return level;}
        int getPriority() const {return priority;}

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority=level+estimate(xDest, yDest)*10; //A*
        }

        // give better priority to going strait instead of diagonally
        void nextLevel(const int & i) // i: direction
        {
             level+=(dir==8?(i%2==0?10:14):10);
        }

        // Estimation function for the remaining distance to the goal.
        const int & estimate(const int & xDest, const int & yDest) const
        {
            static int xd, yd, d;
            xd=xDest-xPos;
            yd=yDest-yPos;

            // Euclidian Distance
            d=static_cast<int>(sqrt(xd*xd+yd*yd));

            // Manhattan distance
            //d=abs(xd)+abs(yd);

            // Chebyshev distance
            //d=max(abs(xd), abs(yd));

            return(d);
        }
};

// Determine priority (in the priority queue)
bool operator<(const node & a, const node & b)
{
  return a.getPriority() > b.getPriority();
}

// A-star algorithm.
// The route returned is a string of direction digits.
string pathFind( const int & xStart, const int & yStart,
                 const int & xFinish, const int & yFinish )
{
   // extern int map[n][m];
    static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the node maps
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dxx[j];
                y+=dyy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dxx[i]; ydy=y+dyy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || mapp[xdx][ydy]==1
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                           pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}






// zmiennie globalne

QString tekst;			 // tekst znajdujący się w polu tekstowym

QString szukany_tekst;	 	 // tekst, który zamienimy na inny
QString zamiennik;		 // tekst na jaki zostanie zamieniona szukana fraza
QString poprzedniaSciezka;

QString tekst1;
int sterowanie=0;

//551123
QString tab_fuzzy[68922];
QString tab_fuzzy_linefallower[68922];
QString tab_przeszkody_plik[1000];
//QString tab_left[552123];
//QString tab_front[552123];
//QString tab_right[552123];
//QString tab_left_engine[552123];
//QString tab_right_engine[552123];
QString lewy_silnik;
QString prawy_silnik;
int fuzzy_left=40;
int fuzzy_front=40;
int fuzzy_right=40;


int fuzzy_left_linefallower=400;
int fuzzy_front_linefallower=400;
int fuzzy_right_linefallower=400;



QString tab_fuzzy_cel[321203];

struct przeszkoda_struct
{
 int x_p;
 int y_p;
 int x_k;
 int y_k;
};

struct meta
{
 int x_pos;
 int y_pos;
};
int numer_przeszkody=0;

przeszkoda_struct  przeszkoda[100];
meta  met;





MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),count(0),count2(0), img(469, 349, QImage::Format_ARGB32),img2(469, 349, QImage::Format_ARGB32),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->pushButton->setIcon(QIcon(":/images/stop.png"));
    ui->pushButton_2->setIcon(QIcon(":/images/down_red.png"));
    ui->pushButton_3->setIcon(QIcon(":/images/right_red.png"));
    ui->pushButton_4->setIcon(QIcon(":/images/left_red.png"));
    ui->pushButton_5->setIcon(QIcon(":/images/up_red.png"));


    for(int i=0;i<n;i++)
           {
            for(int j=0;j<m;j++)
               moja_mapa[i][j]=0;
           }


    start_robota.x_pos=0;
    start_robota.y_pos=0;

get_addres();
    initSocket();

//QString alama="0";
//int alama2=0;

//qDebug() << "wiadomosc  "<<ShellExecute(0, L"runas", L"netsh", L"wlan show interfaces", 0, SW_SHOWNORMAL);
//QProcess(0, L"run", L"netsh", L"wlan show interfaces", 0, SW_SHOWNORMAL);
//printf("asdasdasdasda            %d ", ShellExecute(0,L"netsh" ,L"netsh", L"wlan show interfaces | grep Signal", 0, SW_SHOWNORMAL));
//ui->lineEdit->setText(alama);




vListIPaddress();



scene = new QGraphicsScene(this);
scene2 = new QGraphicsScene(this);
/*
QPen pen(Qt::gray, 1);
for(int i=0;i<=340;i=i+20)
{
scene->addLine(0,i,467,i,pen);
}
for(int i=0;i<=450;i=i+20)
{
scene->addLine(i,0,i,340,pen);
}

*/


ui->graphicsView->setScene(scene);
ui->graphicsView_2->setScene(scene2);

color = qRgb(255, 0, 0);
color_robot = qRgb(255, 0,255 );
color_robot_black = qRgb(0, 70, 20 );
colorlline= qRgb(128, 128, 128);
colorllinewhite= qRgb(255, 255, 255);
colorrect= qRgb(0, 0, 255);
color_meta= qRgb(180, 100, 0);
color_start= qRgb(0, 255, 0);
//color_check_point= qRgb(255, 150, 0);
color_check_point= qRgb(180, 0, 255);
qDebug() << "1:" ;

drawCross(colorlline);

lastpoint_robot.setX(1);
lastpoint_robot.setY(1);
lastpoint_robot2.setX(1);
lastpoint_robot2.setY(1);
//QWebView view;
//    view.show();
//    view.load(QUrl("http://google.com"));
//ui->graphicsView_2->setScene();
QString adres_robota_kamera;
adres_robota_kamera="http://192.168.137.240:8081";

//adres_robota_kamera="http://192.168.100.72:8765";

//ui->web2->load(QUrl("http://192.168.137.240/html/"));
ui->web2->load(QUrl("http://192.168.199.72/html/"));
//ui->web2->load(QUrl("http://192.168.2.72:8765"));
//ui->web2->load(QUrl("http://"+IPaddress+ ":8765"));

//ui->web2->load(QUrl(adres_robota_kamera));
fuzylogic();

wczytaj_dane_z_pliku_omijanie_przeszkod();
wczytaj_dane_z_pliku_linefallower();

//ui->lineEditPort->setText("192.168.2.72:5555");
timer = new QTimer(this);
QObject::connect(timer, SIGNAL(timeout()), this, SLOT(Wyslij_robot()));
timer->start(1000); //time specified in ms


drawText();
rysujRadar_siatka();


kierunek_jazdy_po_prostej[0]=0;
odleglosc_jazdy_po_prostej[0]=0;


ui->lcdNumber->display(27);
}

MainWindow::~MainWindow()
{
    delete ui;
}




class Sleeper : public QThread
{
public:
    static void usleep(unsigned long usecs){QThread::usleep(usecs);}
    static void msleep(unsigned long msecs){QThread::msleep(msecs);}
    static void sleep(unsigned long secs){QThread::sleep(secs);}
};



void MainWindow::stan_polaczenia()
{


// Get WLAN information for available networks
process.start("netsh wlan show interfaces");   // Find wireless access points
process.waitForFinished();
QString wirelessInfo = process.readAll();

ui->lineEdit->setText(wirelessInfo.section(' ', 307,307));
}

/*
void MainWindow::stan_polaczenia()
{


// Get WLAN information for available networks
process.start("netsh wlan show interfaces");   // Find wireless access points
process.waitForFinished();
QString wirelessInfo = process.readAll();

process.start("netsh interface ip show config ");   // Find wireless access points
process.start("netsh wlan show interfaces");   // Find wireless access points
//process.start("netsh wlan show all");   // Find wireless access points
process.waitForFinished();
QString wirelessInfo2 = process.readAll();


//qDebug() << "wiadomosc1  "<<wirelessInfo2;
//qDebug() << "wiadomosc3  "<<wirelessInfo2.size();
//qDebug() << "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa  ";
//qDebug() << "wiadomosc3  "<<wirelessInfo2.section(' ', 307,307);


QStringList wifi2 = wirelessInfo2.split(" ", QString::SkipEmptyParts);
//qDebug() << "wiadomosc2  "<<wifi2.size();
//qDebug() << "wiadomosc22  "<<wifi2.indexOf("192.168.2.3\n");
//qDebug() << "wiadomosc223  "<<wifi2.;       // returns 4
//qDebug() << "asd:  "<< wifi2[119].contains("Wi-Fi", Qt::CaseInsensitive);    // returns true

for(int i=0; i<wifi2.size(); i++)
{
    if(wifi2[i].contains("Wi-Fi", Qt::CaseInsensitive))
    {
    //qDebug() << "ww:  " << i << " adrs: "<<wifi2[i];
   // qDebug() << "ww:  " << i+6 << " adrs: "<<wifi2[i+6];
    adrespc=wifi2[i+6];
    }
}

//qDebug() << "wiadomosc34  "<<adrespc.section(".",0,3);
//qDebug() << "wiadomosc34  "<<adrespc.remove(' ');
adrespc=adrespc.left(adrespc.length()-2);
//qDebug() << "wiadomosc34  "<<adrespc;
//ui->lineEdit->setText(adrespc.replace(" ",""));




ui->lineEdit->setText(wirelessInfo.section(' ', 307,307));
}
*/



void MainWindow::polacz_z_robotem()
{


process.start("netsh interface ip show config ");   // Find wireless access points
process.start("netsh wlan show interfaces");   // Find wireless access points
process.waitForFinished();
QString wirelessInfo2 = process.readAll();



QStringList wifi2 = wirelessInfo2.split(" ", QString::SkipEmptyParts);

for(int i=0; i<wifi2.size(); i++)
{
    if(wifi2[i].contains("Wi-Fi", Qt::CaseInsensitive))
    {
    adrespc=wifi2[i+6];
    }
}

adrespc=adrespc.left(adrespc.length()-2);


}





void MainWindow::Wyslij_robot()
{
   // get_addres();
     //  initSocket();
  // stan_polaczenia();
   readPendingDatagrams();
    qDebug() << "Wyslij_robot ";
  //  if(finish==0)
   //     fuzylogic_cel();

}



void MainWindow::get_addres()
{
    //nPort = 0;
    //IPaddress
    ui->lineEditPort->setText(QString::number(nPort));
    connect(ui->lineEditPort,SIGNAL(textChanged(QString)),this,SLOT(updateUdpPort(QString)));

}

void MainWindow::initSocket()
{
    udpSocket = new QUdpSocket(this);
//    udpSocket->bind(QHostAddress::LocalHost, 33333);
    udpSocket->bind(QHostAddress::Any, nPort);

    connect(udpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
//    192.168.2.72:5555

}
void MainWindow::readPendingDatagrams()
{
    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        udpSocket->readDatagram(datagram.data(), datagram.size(),
                                &sender, &senderPort);

        processTheDatagram(datagram,sender);
    }
}

void MainWindow::processTheDatagram(QByteArray datagram, QHostAddress qha){
    bool ok;
    int n = datagram.toInt(&ok,10);
    QString qsz;
    QString wiadomosc;
    if(ok){
        emit newAngle(n);
        if ( (n < 5) || (n > 70))
        {
            qsz = trUtf8("%1 <font color = red>%2</color>").
                  arg(qha.toString()).
                  arg(datagram.data());
            wiadomosc=datagram.data();
        }
        else
        {
            qsz = trUtf8("%1 <font color = blue>%2</color>").
                  arg(qha.toString()).
                  arg(datagram.data());
            wiadomosc=datagram.data();
        }
    }else
    {
        qsz = trUtf8("%1 <font color = red>%2</color>").
              arg(qha.toString()).
              arg(datagram.data());
        wiadomosc=datagram.data();
    }
    ui->plainTextEdit->appendHtml(qsz);
    odbierzDane(wiadomosc);

}

void MainWindow::updateUdpPort(QString qsz){
    bool ok;
    QString nowyIP;
    QString nowyPort;

    nowyIP=qsz.section(':',0,0);
    nowyPort=qsz.section(':',1,1);
    qint16 n = nowyPort.toInt(&ok,10);
 qDebug() << "nowyPort " << nowyPort;
  qDebug() << "nowyIP " << nowyIP;
    if(ok){
        disconnect(udpSocket);
        udpSocket->abort();
        delete udpSocket;
        nPort = n;
        IPaddress=nowyIP;
        qDebug() << "nPort " << nPort;
         qDebug() << "IPaddress " << IPaddress;
        initSocket();
    }
}

void MainWindow::vListIPaddress(){
    QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();

    // use the first non-localhost IPv4 address
    for (int i = 0; i < ipAddressesList.size(); ++i) {
        if (/*ipAddressesList.at != QHostAddress::LocalHost &&*/
            ipAddressesList.at(i).protocol() == QAbstractSocket::IPv4Protocol)

            {
                qDebug() << trUtf8("ipAddress : %1").
                        arg(ipAddressesList.at(i).toString());


/*
                QList<QNetworkInterface> ifaces = QNetworkInterface::allInterfaces();
                QNetworkInterface iface = ifaces.at(i);
                qDebug() << trUtf8("ipAddress1 : %1").arg(iface.hardwareAddress());
                qDebug() << trUtf8("ipAddress2 : %1").arg(iface.humanReadableName());
                qDebug() << trUtf8("ipAddress3 : %1").arg(iface.profile());
               // qDebug() << iface.addressEntries().at(i).ip().toString()
               //                          << " / " << iface.addressEntries().at(i).netmask().toString() << endl;

              //  QNetworkInterface eth1Ip = QNetworkInterface::interfaceFromName("eth1");
                QList<QNetworkAddressEntry> entries = iface.addressEntries();
                //if (!entries.isEmpty()) {
                    QNetworkAddressEntry entry = entries.first();
                //    qDebug() <<  "a1  " << entry.ip().toString();
                 //   qDebug() <<  "a2  "<< entries.at(i).ip().toString();
                //    qDebug() <<  "a2  "<< entries.at(i).netmask().toString();
*/
               // }

               // QNetworkInterface ipAddressesList2 = QNetworkInterface::interfaceFromName("Piotrek");
               //  QList<QHostAddress>  ipAddressesList3 = QNetworkInterfacePrivate;
                //        qDebug() << trUtf8("asdasdasd : %1").arg(ipAddressesList2.at(i).toString());
                //break;

                 //       foreach (const QHostAddress &address, QNetworkInterface::allAddresses()) {
                 //           if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
                 //                qDebug() << "sdasdasd sdasda "<< address.toString();
                 //       }




            }
    }
}







void MainWindow::odbierzDane(QString wiadomosc)
{
QString wybierz,info;


wybierz = wiadomosc.section(',', 0, 0);   // str == "1"
info    = wiadomosc.section(',', 1, 1);   // str == "24stopnie"

//qDebug() << "wiadomosc  "<<wiadomosc;
qDebug() << "wybierz  "<<wybierz;
qDebug() << "info  "<<info;

double x1,y1;
double vel=info.toDouble();
int velocity=vel;
int  digit=0;
int a1,a2,a3,a4;
int numer[5],dystans[5],kolor[5];
/*
for (int var = 0; var < info.length(); ++var) {
    bool ok;
    if (info.at(var).isDigit()){
         digit = info.at(var).digitValue();
        //DO SOMETHING HERE WITH THE DIGIT
    }
}
*/
bool ok;

switch (wybierz.toInt()) {
case 1:
    ui->lineEdit->setText(info);
    break;
case 2:
    ui->lineEdit_2->setText(info);
    break;
case 3:
    ui->lineEdit_3->setText(info);
    break;
case 4:
    ui->lineEdit_4->setText(info);
    break;
case 5:
    ui->lineEdit_5->setText(info);
    break;
case 6:
    ui->lineEdit_6->setText(info);
    break;
case 7:
    ui->lineEdit_7->setText(info);
    break;
case 8:
    ui->lineEdit_8->setText(info);
    break;
case 9:
    ui->lineEdit_9->setText(info);
    break;
case 10:
   //info=QString::number(info);
    //info=QString("%1").arg(info.toInt());
   // int velocity=20;
    ui->lcdNumber_2->display(velocity);
    break;
case 11:
    ui->lcdNumber_3->display(info.toInt());
    break;
case 12:
    ui->lcdNumber_4->display(info.toInt());
    break;
case 13:

    fuzzy_left=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
    fuzzy_front=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
    fuzzy_right=info.section('*', 2, 2).toDouble(&ok);   // str == "1"
    break;
case 14:

// jazda do celu

    flaga_omijanie_przeszkod=false;
    flaga_jazda_do_celu=true;
    flaga_omijanie_przeszkod_jazda_do_celu=false;
    flaga_planowanie_jazda_do_celu_fuzzy=false;
            a1=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
   poz_enkoder_left=a1;
            a2=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
             poz_enkoder_right=a2;
    qDebug() << "aaa   x1 "<<a1 << " y1 " << a2;

    if(x>=0 && y<=0)
        if(x<=width && y>=-height)
            rysujTraseRobot( x,  -y);

    fuzylogic_cel();

    break;
case 15:

    xg=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
    yg=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
    break;
case 16:


   a3= x1=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
   a4= y1=info.section('*', 1, 1).toDouble(&ok);   // str == "1"

    rysujTraseRobot( a3,  a4);
   // rysujTraseRobot( x1,  y1);
     qDebug() << "x y   "<<x1 << " " << y1;
    break;
case 17:
    numer[0]=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
    dystans_omijanie[0]=dystans[0]=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
    for(int i=1;i<5;i++)
    {
    numer[i]=info.section('*', i*2, i*2).toDouble(&ok);   // str == "1"
    dystans_omijanie[i]=dystans[i]=info.section('*', i*2+1, i*2+1).toDouble(&ok);   // str == "1"
    }

    radar_hcsr04(numer,dystans );
    break;
case 18:

    if(flaga_omijanie_przeszkod)
      {

    numer[0]=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
    dystans[0]=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
    for(int i=1;i<5;i++)
    {
    numer[i]=info.section('*', i*2, i*2).toDouble(&ok);   // str == "1"
    dystans[i]=info.section('*', i*2+1, i*2+1).toDouble(&ok);   // str == "1"
    }
    if(dystans[0] >=40)
        dystans[0]=40;
    if(dystans[1] >=40)
        dystans[1]=40;
    if(dystans[2] >=40)
        dystans[2]=40;
    if(dystans[3] >=40)
        dystans[3]=40;
    if(dystans[4] >=40)
        dystans[4]=40;

    // od lewego 5 4 3 2 1
//   qDebug() << "1  "<<dystans[0]; // od lewego 5
//   qDebug() << "2  "<<dystans[1]; // 4
//   qDebug() << "3  "<<dystans[2]; //
//   qDebug() << "4  "<<dystans[3]; //
//   qDebug() << "5  "<<dystans[4]; //
    if(dystans[0] <=33 || dystans[1] <=33 || dystans[2] <=33 || dystans[3] <=33 || dystans[4] <=33  )
    {
        if(dystans[0] >=7 && dystans[1] >=7 && dystans[2] >=7 && dystans[3] >=7 && dystans[4] >=7 )
        {

   //  fuzzy_left=dystans[2];
    // fuzzy_front=dystans[1];
   //  fuzzy_right=dystans[0];
                      fuzzy_left=qMin(dystans[3],dystans[4]);
                      fuzzy_front=dystans[2];
                      fuzzy_right=qMin(dystans[0],dystans[1]);
        }
        else
        {
            if(dystans[0] >=5 && dystans[1] >=5 && dystans[2] >=5 && dystans[3] >=5 && dystans[4] >=5 )
            {
             uaktualnij_dane("2,0");
            }
            else
            {
            if(licznik_omijanie_przeszkod>=2)
            {
            uaktualnij_dane("5,0");
            uaktualnij_dane("12,3");
            licznik_omijanie_przeszkod=0;
            }
            }
            licznik_omijanie_przeszkod++;
        }
    }
    else
    {
        fuzzy_left=40;
        fuzzy_front=40;
        fuzzy_right=40;
    }

     fuzylogic_jedz();

    }
    break;
case 19:


            a1=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
            poz_enkoder_left=a1;
            a2=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
            poz_enkoder_right=a2;
    fuzylogic_jedz_do_celu_po_prostej();
    break;
case 20:



    a1=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
poz_enkoder_left=a1;
    a2=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
     poz_enkoder_right=a2;
qDebug() << "aaa   x1 "<<a1 << " y1 " << a2;
flaga_omijanie_przeszkod=false;

if(x>=0 && y<=0)
if(x<=width && y>=-height)
    rysujTraseRobot( x,  -y);

if(dystans_omijanie[0] <=35 || dystans_omijanie[1] <=35 || dystans_omijanie[2] <=35 || dystans_omijanie[3] <=35 || dystans_omijanie[4] <=35  )
    {


  //  qDebug() << "asdasdasdasdasasdasdasdasddasdasdasda";

    if(dystans_omijanie[0] >=40)
        dystans_omijanie[0]=40;
    if(dystans_omijanie[1] >=40)
        dystans_omijanie[1]=40;
    if(dystans_omijanie[2] >=40)
        dystans_omijanie[2]=40;
    if(dystans_omijanie[3] >=40)
        dystans_omijanie[3]=40;
    if(dystans_omijanie[4] >=40)
        dystans_omijanie[4]=40;


    if(dystans_omijanie[0] <=35 || dystans_omijanie[1] <=35 || dystans_omijanie[2] <=35 || dystans_omijanie[3] <=35 || dystans_omijanie[4] <=35  )
    {
        if(dystans_omijanie[0] >=4 && dystans_omijanie[1] >=4 && dystans_omijanie[2] >=4 && dystans_omijanie[3] >=4 && dystans_omijanie[4] >=4 )
        {

   //  fuzzy_left=dystans[2];
    // fuzzy_front=dystans[1];
   //  fuzzy_right=dystans[0];
                      fuzzy_left=qMin(dystans_omijanie[3],dystans_omijanie[4]);
                      fuzzy_front=dystans_omijanie[2];
                      fuzzy_right=qMin(dystans_omijanie[0],dystans_omijanie[1]);
        }
        else
        {
            if(licznik_omijanie_przeszkod>=1)
            {
          //  uaktualnij_dane("5,0");
            uaktualnij_dane("17,3");
            licznik_omijanie_przeszkod=0;
            }
            licznik_omijanie_przeszkod++;
        }
    }
    else
    {
        fuzzy_left=40;
        fuzzy_front=40;
        fuzzy_right=40;
    }

     fuzylogic_jedz();




    }
else
    {
  //  flaga_omijanie_przeszkod=false;
 //   qDebug() << "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

    fuzylogic_cel();

    }




    break;

case 21:

    on_pushButton_clicked();
    break;

case 22:
    ui->lineEdit_10->setText(info);
    break;

case 23:

// jazda do celu
    flaga_omijanie_przeszkod=false;
    flaga_jazda_do_celu=false;
    flaga_omijanie_przeszkod_jazda_do_celu=false;
    flaga_planowanie_jazda_do_celu_fuzzy=true;
            a1=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
   poz_enkoder_left=a1;
            a2=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
             poz_enkoder_right=a2;
    qDebug() << "aaa   x1 "<<a1 << " y1 " << a2;

    if(x>=0 && y<=0)
        if(x<=width && y>=-height)
            rysujTraseRobot( x,  -y);

 //   if(x>=0 && y<=0)
 //       if(x<=460 && y>=-340)
 //           rysujTraseRobot( x,  -y);
    if(z<9)
    {
        if(licznik_xg_yg<licznik_planowanie)
        {
            uaktualnij_dane("5,0");
            break;
        }

    xg=xg_planowanie[licznik_planowanie];
    yg=yg_planowanie[licznik_planowanie];
    licznik_planowanie++;

    }

    fuzylogic_cel();

    break;

case 24:

    if(flaga_linefallower)
      {

    numer[0]=info.section('*', 0, 0).toDouble(&ok);   // str == "1"
    kolor[0]=info.section('*', 1, 1).toDouble(&ok);   // str == "1"
    for(int i=1;i<5;i++)
    {
    numer[i]=info.section('*', i*2, i*2).toDouble(&ok);   // str == "1"
    kolor[i]=info.section('*', i*2+1, i*2+1).toDouble(&ok);   // str == "1"
    }
    if(kolor[0] >=400)
        kolor[0]=400;
    if(kolor[1] >=400)
        kolor[1]=400;
    if(kolor[2] >=400)
        kolor[2]=400;
    if(kolor[3] >=400)
        kolor[3]=400;
    if(kolor[4] >=400)
        kolor[4]=400;

    // od lewego 5 4 3 2 1
  // qDebug() << "1  "<<kolor[0]; // od lewego 5
 //  qDebug() << "2  "<<kolor[1]; // 4
  // qDebug() << "3  "<<kolor[2]; //
  // qDebug() << "4  "<<kolor[3]; //
  // qDebug() << "5  "<<kolor[4]; //

   fuzzy_left_linefallower=qMax(kolor[3],kolor[4]);
   fuzzy_front_linefallower=kolor[2];
   fuzzy_right_linefallower=qMax(kolor[0],kolor[1]);

  /*  if(kolor[0] <=300 || kolor[1] <=300 || kolor[2] <=300 || kolor[3] <=300 || kolor[4] <=300  )
    {
        if(kolor[0] >=7 && kolor[1] >=7 && kolor[2] >=7 && kolor[3] >=7 && kolor[4] >=7 )
        {

                      fuzzy_left_linefallower=qMin(dystans[3],dystans[4]);
                      fuzzy_front_linefallower=dystans[2];
                      fuzzy_right_linefallower=qMin(dystans[0],dystans[1]);
        }


    }
    else
    {
        fuzzy_left=40;
        fuzzy_front=40;
        fuzzy_right=40;
    }
*/


     fuzylogic_linefallower();

    }
    break;


default:
    break;
}






}















void MainWindow::on_pushButton_5_clicked()
{


  /*  if(ui->pushButton_5->text() == QString("push up"))
            ui->pushButton_5->setText(tr("up"));
        else
            ui->pushButton_5->setText(tr("push up"));
            */
    QString text = ui->sterowanie->text();
    ui->sterowanie->setText("jedzie do przodu");
   sterowanie=1;
   uaktualnij_dane("5,0");
    uaktualnij_dane("1,3");




}

void MainWindow::on_pushButton_clicked()
{

    QString text = ui->sterowanie->text();
    ui->sterowanie->setText("stop");
    sterowanie=0;
    flaga_omijanie_przeszkod=false;
    flaga_jazda_do_celu=false;
    flaga_omijanie_przeszkod_jazda_do_celu=false;
    flaga_planowanie_jazda_do_celu_fuzzy=false;
    flaga_linefallower=false;

uaktualnij_dane("5,0");

}

void MainWindow::on_pushButton_4_clicked()
{
    QString text = ui->sterowanie->text();
    ui->sterowanie->setText("jedzie w lewo");
    sterowanie=3;
    uaktualnij_dane("5,0");
    uaktualnij_dane("2,3");

}

void MainWindow::on_pushButton_3_clicked()
{
    QString text = ui->sterowanie->text();
    ui->sterowanie->setText("jedzie w prawo");
    sterowanie=4;
    uaktualnij_dane("5,0");
    uaktualnij_dane("3,3");
}

void MainWindow::on_pushButton_2_clicked()
{
    QString text = ui->sterowanie->text();
    ui->sterowanie->setText("jedzie do tyłu");
    sterowanie=2;
    uaktualnij_dane("5,0");
    uaktualnij_dane("4,3");
}

void MainWindow::on_verticalSlider_sliderMoved(int position)
{
    //QString text = ui->lcdNumber->text();
    if(position%1==0)
    {
    ui->lcdNumber->display(position);


    QString tmp = QString("20,%1")
                         .arg(position);

    uaktualnij_dane(tmp);
    }
}


void MainWindow::on_pushButton_6_clicked()
{

    //MyUDP sender;
  //  MainWindow receiver;
  //  QObject::connect(&my, SIGNAL(wyslij_do_mainwindow(QString,QString,QString)), &receiver, SLOT(odczytDanych(QString,QString,QString)));
    //my.uaktualnij_dane("Polaczono z robotem");
  //  my.readyRead();
   // my.fireSignal();

polacz_z_robotem();
QString s="9,";
s+=adrespc;
uaktualnij_dane(s);

//QString adres_robota_kamera;
//adres_robota_kamera="http://"+IPaddress+":8765";
//ui->web2->load(QUrl(adres_robota_kamera));

//qDebug() << "ww:  " << s;
//ui->lineEditPolacz->setText(tekst1);


}











void MainWindow::on_actionOtw_rz_triggered()
{
    // QFileDialog::getOpenFileName([rodzic (parent)],[tytuł nagłówka],
    // [ścieżka od której rozpoczyna się wyszukiwanie pliku], [rozszerzenia wyszukiwanych plików])
    QString fileName = QFileDialog::getOpenFileName(this,tr("Otwórz..."), "/home/", tr("Pliki txt (*.txt )"));
    QFile plik(fileName);
    qDebug() << "wwwwww  "<<fileName;
    sciezka_do_pliku_przeszkody=fileName;
        // możemy tylko czytać dane, oraz wczytujemy je jako tekst:
        if(!plik.open(QIODevice::ReadOnly | QIODevice::Text))
            return;			 // jeżeli nie udało się otworzyć pliku: przerwij wczytywanie pliku

        // czyścimy wcześniej zapełnioną zmienną tekstową
        tekst.clear();

        QTextStream stream(&plik);

        // czytamy wszystkie dane
        tekst = stream.readAll();

        // umieszczamy je wewnątrz text boxa
        ui->textEdit_2->setText(tekst);

        plik.close();

        // na końcu "ciała" funkcji void MainWindow::otworz_plik()
        poprzedniaSciezka = fileName;
}


void MainWindow::zapisz_plik(QString fileName)
{
   QString tekst_chwilowy;
   int licznik_lini_tekstu;
    tekst_chwilowy = ui->textEdit_2->toPlainText();

    licznik_lini_tekstu= ui->textEdit_2->document()->blockCount();
    tekst = QString::number(licznik_lini_tekstu);
    tekst += "\n";
    tekst += tekst_chwilowy;
   // qDebug() << "wwwwww  "<< ui->textEdit_2->lineWrapColumnOrWidth();
   // qDebug() << "wwwwww  "<< ui->textEdit_2->lineWidth();
    //qDebug() << "wwwwww  "<< ui->textEdit_2->document()->blockCount();
    if(fileName.isEmpty())
        fileName = QFileDialog::getSaveFileName
                (this,tr("Zapisz plik jako..."),tr("/home/"),tr("Pliki tekstowe (*.txt)"));

    if(fileName.isEmpty())
        return;

    QFile plik(fileName);
    plik.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);

    QTextStream stream(&plik);
    stream << tekst;

    poprzedniaSciezka = fileName;

    plik.close();
}



void MainWindow::zapisz_plik_jako( )
{

    QString fileName = QFileDialog::getSaveFileName(this,tr("Otwórz..."), "/home/", tr("Pliki txt (*.txt)"));

    QString tekst_chwilowy;
    int licznik_lini_tekstu;
     tekst_chwilowy = ui->textEdit_2->toPlainText();

     licznik_lini_tekstu= ui->textEdit_2->document()->blockCount();
     tekst = QString::number(licznik_lini_tekstu);
     tekst += "\n";
     tekst += tekst_chwilowy;

    //tekst = ui->plainTextEdit->toPlainText();
     // tekst = ui->textEdit_2->toPlainText();
    if(fileName.isEmpty())
        fileName = QFileDialog::getSaveFileName
                (this,tr("Zapisz plik jako..."),tr("/home/"),tr("Pliki tekstowe (*.txt)"));

    if(fileName.isEmpty())
        return;

    QFile plik(fileName);
    plik.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);

    QTextStream stream(&plik);
    stream << tekst;

    poprzedniaSciezka = fileName;

    plik.close();
}

void MainWindow::on_actionZapisz_triggered()
{
    zapisz_plik(poprzedniaSciezka);
}

void MainWindow::on_actionZapisz_jako_triggered()
{
    zapisz_plik_jako();
}

void MainWindow::on_actionZamknij_triggered()
{

    qApp->exit();
}

















void MainWindow::uaktualnij_dane(const QString& dane)
{
 QByteArray Dane;
 Dane.append(dane);
Sleeper::msleep(10);
//udpSocket->writeDatagram(Dane,QHostAddress("192.168.0.255"),5001);
udpSocket->writeDatagram(Dane,QHostAddress(IPaddress),nPort+1);
qDebug() << "Message from: bufforu " << Dane;


}



void MainWindow::mousePressEvent(QMouseEvent *event)
{

    if(ui->checkBox_2->isChecked())
    {
        rysujTrase(event);
    }
    if(ui->checkBox_3->isChecked())
    {
      //  rysujPrzeszkode(20,20,100,100,colorrect);
    }



}




void MainWindow::drawCross(QRgb colorllinee)
{

    if(ui->checkBox->isChecked())
    {



    for(int i=0;i<=width;i++)
    {
        for(int j=0;j<=height;j=j+ui->spinBox_2->value())
        {
        img.setPixel(i, j,colorllinee );

        }

    }
    for(int i=0;i<=height;i++)
    {
        for(int j=0;j<=width;j=j+ui->spinBox_2->value())
        {
        img.setPixel(j, i,colorllinee );

        }

    }
scene->addPixmap(QPixmap::fromImage(img));
scene->backgroundBrush();

 }

}




void MainWindow::rysujPrzeszkode(int xx,int yy,int szer,int wys,QRgb colorrect)
{
    if(xx<=width-20 && yy <= height && xx>0 && yy>0)
    {
int x,y;
szer=szer+xx;
wys=wys+yy;
qDebug() << "asdasdasd12312312:" ;
   for( x=xx;x<=szer;x++)
    {
        for(y=yy ;y<=wys;y++)
        {
        img.setPixel(x, y,colorrect );
       // img.setPixel(y, x,colorlline );
        moja_mapa[x][y]=1;
        }

    }
   for( x=xx-10;x<=szer+10;x++)
    {
        for(y=yy-10 ;y<=wys+10;y++)
        {

        moja_mapa[x][y]=1;
        }

    }
scene->addPixmap(QPixmap::fromImage(img));
scene->backgroundBrush();

    }
}



void MainWindow::rysujTrase(QMouseEvent *event)
{



    qDebug() << "1aa:" ;
    switch(event->button())
    {
    qDebug() << "2aa:" ;
        case Qt::LeftButton:
            count++;
qDebug() << "2a:" ;
            if(1 == count)
            {
                if(lastpoint.x()>=0 && lastpoint.x()<=width && lastpoint.y()>=0 && lastpoint.y()<=height)
                {
                p1 = ui->graphicsView->mapFromGlobal(event->globalPos());



                p1=lastpoint;
                if(count2==0)
                {
                 lastpoint=ui->graphicsView->mapFromGlobal(event->globalPos());
                 count2=100;
                 p1=lastpoint;
                }
                qDebug() << "1bxxxxxxxxxxxxxxxxxxxxxxxx:" ;
                }
                else
                {
                p1 = ui->graphicsView->mapFromGlobal(event->globalPos());



                }
                //p1 = ui->graphicsView->mapFromGlobal(event->pos())
                qDebug() << "1a:" ;
            }
            else if(2 == count)
            {
                p2 = ui->graphicsView->mapFromGlobal(event->globalPos());


                int k=1;
                k=ui->spinBox_3->value();

                int xx1=p1.x();
                int yy1=p1.y();
                qDebug() << "2aap2xxx:   " <<xx1;
                qDebug() << "2aap2xxx:   " <<yy1;
                p1.setX(p1.x()-xx1%k);
                p1.setY(p1.y()-yy1%k);
                qDebug() << "3aap2xxx:   " <<p1.x();
                qDebug() << "3aap2xxx:   " <<p1.y();

                pomoc_point.setX(abs(p2.x()-p1.x()));
                pomoc_point.setY( abs(p2.y()-p1.y()));
                if(pomoc_point.x()>=pomoc_point.y())
                {
                    p2.setY(p1.y());

                }
                else
                {
                  p2.setX(p1.x());

                }


                lastpoint=p2;
                //p2 = ui->graphicsView->x();
qDebug() << "3a:" ;
                drawLine();
                reset();
                scene->addPixmap(QPixmap::fromImage(img));
            }

            break;

        default:
            ;
            img.fill(Qt::white);
    }



}





void MainWindow::drawLine()
{

drawCross(colorlline);
    int pom=0;
    //I assume we have a coordinate system with the origin in the upper left corner
    //with the positive axes down and to the right, this is set up in the ctor



    int k=1;
    k=ui->spinBox_3->value();


    qDebug() << "1aap2:   " <<k;
    int xx=p2.x();
    int yy=p2.y();
    qDebug() << "2aap2:   " <<xx;
    qDebug() << "2aap2:   " <<yy;
    p2.setX(p2.x()-xx%k);
    p2.setY(p2.y()-yy%k);
    qDebug() << "3aap2:   " <<p2.x();
    qDebug() << "3aap2:   " <<p2.y();

poz_x_jada_do_celu_po_prostej=p2.x();
poz_y_jada_do_celu_po_prostej=p2.y();
kierunek_jazdy();

    //first point
    QPoint f = p1.toPoint();
    //last point
    QPoint l = p2.toPoint();
qDebug() << "4:" ;
    //vertical line
    if(f.x() == l.x())
    {
        qDebug() << "vertical line";

        if(f.y() > l.y())
        {
            qSwap(f, l);
        }

        for(int y=f.y(); y<=l.y(); y++)
        {

            if(pom%ui->spinBox->value()==0)
            {
               qDebug() << "x:" << f.x() << "y:" << y;
               QString tmp = QString("%1,%2,%3,%4").arg("x").arg(f.x()).arg("y").arg(y);
               uaktualnij_dane(tmp);
              // Sleeper::usleep(10);
              // Sleeper::msleep(100);
              // Sleeper::sleep(10);

            }
            pom++;
            img.setPixel(f.x(), y, color);
            img.setPixel(f.x()+1, y+1, color);
            img.setPixel(f.x()-1, y-1, color);
        }

        return;
    }

    //slope
    qreal m = (p2.y() - p1.y())/(p2.x() - p1.x());
    qDebug() << "Slope:" << m;

    //in order to get better effects
    //I should render the line iterating through the longer side
    //the if condition translates into: the rise is higher than the run
    if(m > 1.0 || m < -1.0)
    {
        qDebug() << "the rise is higher than the run";

        m = 1/m;

        //draw the line from the top to bottom
        if(f.y() > l.y())
        {
            qSwap(f, l);
        }

        for(int y=f.y(); y<=l.y(); y++)
        {
            int x = round(m*(y-f.y()) + f.x());

            if(pom%ui->spinBox->value()==0)
            {
               qDebug() << "x:" << x << "y:" << y;
               QString tmp = QString("%1,%2,%3,%4").arg("x").arg(x).arg("y").arg(y);
               uaktualnij_dane(tmp);
              // Sleeper::msleep(100);
            }
            pom++;

            img.setPixel(x, y, color);
            img.setPixel(x+1, y+1, color);
            img.setPixel(x-1, y-1, color);
        }
    }
    else{
        //if the line was drawn right to left, make it so I draw left to right
 if(f.x() > l.x())
        {
            qSwap(f, l);
        }

        for(int x=f.x(); x<=l.x(); x++)
        {
            int y = round(m*(x-f.x()) + f.y());


            if(pom%ui->spinBox->value()==0)
            {
               qDebug() << "x:" << x << "y:" << y;
               QString tmp = QString("%1,%2,%3,%4").arg("x").arg(x).arg("y").arg(y);
               uaktualnij_dane(tmp);
             //  Sleeper::msleep(100);
            }
            pom++;


            img.setPixel(x, y,color );
            img.setPixel(x+1, y+1,color );
            img.setPixel(x-1, y-1,color );

        }
    }

}

void MainWindow::reset()
{


    count = 0;
    scene->clear();

    //count2=0;
    p1 = p2 = QPoint(0, 0);
    drawCross(colorlline);
    scene->addPixmap(QPixmap::fromImage(img));

}







void MainWindow::on_checkBox_toggled(bool checked)
{
    if(checked==true)
    {
        drawCross(colorlline);
    }
    else
    {
        scene->clear();
      //  drawCross(colorllinewhite);

    }

}

void MainWindow::on_checkBox_3_toggled(bool checked)
{


    if(checked==true)
    {

    }
    else
    {

      //  scene->clear();
      //  rysujPrzeszkode(int xx,int yy,int szer,int wys,colorrect);

    }
}

void MainWindow::on_pushButton_11_clicked()
{

    QString dodawanie_przeszkody;
    if(ui->checkBox_3->isChecked())
    {



    int xx,yy,szer,wys;
    xx=ui->spinBox_4->value();
    yy=ui->spinBox_5->value();
    szer=ui->spinBox_6->value();
    wys=ui->spinBox_7->value();
przeszkoda[numer_przeszkody].x_p=xx;
przeszkoda[numer_przeszkody].y_p=yy;
przeszkoda[numer_przeszkody].x_k=xx+szer;
przeszkoda[numer_przeszkody].y_k=xx+wys;
numer_przeszkody++;

    rysujPrzeszkode( xx, yy, szer, wys,colorrect);
drawCross(colorlline);
dodawanie_przeszkody=QString::number(xx)+" "+QString::number(yy)+" "+QString::number(szer)+" "+QString::number(wys);
ui->textEdit_2->append(dodawanie_przeszkody);
    }
}





void MainWindow::fuzylogic()
{


QFile file("C:/Users/lenovo/Documents/PROJEKTY QT/Najlepszy_v2/Generator2_v1/plik_cel_do_460cm_co_1cm_005s.txt");
//QFile file("C:/Users/lenovo/Documents/PROJEKTY QT/Najlepszy_v2/Generator2_v1/plik_do_1m_co_1cm.txt");
if(!file.open(QIODevice::ReadOnly)) {
    QMessageBox::information(0, "error", file.errorString());
}


QString line;
//QStringList fields;
//QString tmp;
//QString fleft,ffront,fright,fleft_engine,fright_engine;
QTextStream in(&file);
int i=0;
while(!in.atEnd()) {
     line = in.readLine();
   //  fields = line.split(",");
  //  model->appendRow(fields);
  //  qDebug() <<i << " "<< line;
  //   tmp = QString("%1").arg(line);


//tab_fuzzy[i] = line;   // str == "1"
tab_fuzzy_cel[i]= line;




   //  tab_left[i] = line.section(' ', 0, 0);   // str == "1"
   //  tab_front[i]    = tmp.section(' ', 1, 1);   // str == "24stopnie"
   //  tab_right[i] = tmp.section(' ', 2, 2);   // str == "1"
   //  tab_left_engine[i]    = tmp.section(' ', 3, 3);   // str == "24stopnie"
   //  tab_right_engine[i] = tmp.section(' ', 4, 4);   // str == "1"


   // tab_left[i]=fleft;
   // tab_right[i]=fright;
   // tab_front[i]=ffront;
  //  tab_left_engine[i]=fleft_engine;
  //  tab_right_engine[i]=fright_engine;

//qDebug() << "tab_left " <<tab_left[500] ;
i++;
}

file.close();


qDebug() << "ok gotowe"  ;

}










void MainWindow::wczytaj_dane_z_pliku_omijanie_przeszkod()
{



QFile file("C:/Users/lenovo/Documents/PROJEKTY QT/Najlepszy_v2/Generator2_v1/plik_do_40cm_co_1cm.txt");
if(!file.open(QIODevice::ReadOnly)) {
    QMessageBox::information(0, "error", file.errorString());
}

QString line;
QTextStream in(&file);
int i=0;
while(!in.atEnd()) {
     line = in.readLine();

tab_fuzzy[i] = line;   // str == "1"
i++;
}

file.close();

qDebug() << "ok gotowe"  ;

}


void MainWindow::wczytaj_dane_z_pliku_linefallower()
{



QFile file("C:/Users/lenovo/Documents/PROJEKTY QT/Najlepszy_v2/Generator2_v1/linefallower.txt");
if(!file.open(QIODevice::ReadOnly)) {
    QMessageBox::information(0, "error", file.errorString());
}

QString line;
QTextStream in(&file);
int i=0;
while(!in.atEnd()) {
     line = in.readLine();

tab_fuzzy_linefallower[i] = line;   // str == "1"
i++;
}

file.close();

qDebug() << "ok gotowe"  ;

}


void MainWindow::wczytaj_dane_z_pliku_przeszkody()
{

QFile file(sciezka_do_pliku_przeszkody);
//QFile file("C:/Users/lenovo/Documents/PROJEKTY QT/Najlepszy_v2/Generator2_v1/przeszkody.txt");
if(!file.open(QIODevice::ReadOnly)) {
    QMessageBox::information(0, "error", file.errorString());
}

QString line;
QTextStream in(&file);
int i=0;
while(!in.atEnd()) {
     line = in.readLine();

tab_przeszkody_plik[i] = line;   // str == "1"
qDebug() << " przeszkoda " << tab_przeszkody_plik[i] ;
i++;
}

file.close();

qDebug() << "ok gotowe przeszkody"  ;

}






//int liczniki=0;
void MainWindow::fuzylogic_jedz()
{

    /*
    int fuzzy_left=40;
    int fuzzy_front=40;
    int fuzzy_right=40;


    if(liczniki==0)
    {
         fuzzy_left=40;
         fuzzy_front=40;
         fuzzy_right=40;

    }
    if(liczniki==1)
    {
         fuzzy_left=40;
         fuzzy_front=40;
         fuzzy_right=39;

    }
    if(liczniki==2)
    {
         fuzzy_left=40;
         fuzzy_front=39;
         fuzzy_right=40;

    }
    if(liczniki==3)
    {
         fuzzy_left=40;
         fuzzy_front=39;
         fuzzy_right=39;

    }
    if(liczniki==4)
    {
         fuzzy_left=39;
         fuzzy_front=39;
         fuzzy_right=39;

    }
    if(liczniki==5)
    {
         fuzzy_left=39;
         fuzzy_front=39;
         fuzzy_right=40;

    }
    if(liczniki==6)
    {
         fuzzy_left=39;
         fuzzy_front=40;
         fuzzy_right=39;

    }
    if(liczniki==7)
    {
         fuzzy_left=39;
         fuzzy_front=40;
         fuzzy_right=40;

    }

    if(liczniki==8)
    {
         fuzzy_left=0;
         fuzzy_front=0;
         fuzzy_right=0;

    }
    if(liczniki==9)
    {
         fuzzy_left=0;
         fuzzy_front=0;
         fuzzy_right=1;

    }
    if(liczniki==10)
    {
         fuzzy_left=0;
         fuzzy_front=1;
         fuzzy_right=0;

    }
    if(liczniki==11)
    {
         fuzzy_left=0;
         fuzzy_front=1;
         fuzzy_right=1;

    }
    if(liczniki==12)
    {
         fuzzy_left=1;
         fuzzy_front=0;
         fuzzy_right=0;

    }
    if(liczniki==13)
    {
         fuzzy_left=1;
         fuzzy_front=0;
         fuzzy_right=1;

    }
    if(liczniki==14)
    {
         fuzzy_left=1;
         fuzzy_front=1;
         fuzzy_right=0;

    }
    if(liczniki==15)
    {
         fuzzy_left=1;
         fuzzy_front=1;
         fuzzy_right=1;

    }
    if(liczniki==16)
    {
         fuzzy_left=40;
         fuzzy_front=0;
         fuzzy_right=0;

    }
    if(liczniki==17)
    {
         fuzzy_left=40;
         fuzzy_front=0;
         fuzzy_right=1;

    }
    if(liczniki==18)
    {
         fuzzy_left=40;
         fuzzy_front=1;
         fuzzy_right=1;

    }
    if(liczniki==19)
    {
         fuzzy_left=40;
         fuzzy_front=40;
         fuzzy_right=0;

    }
    if(liczniki==20)
    {
         fuzzy_left=40;
         fuzzy_front=40;
         fuzzy_right=1;

    }
    if(liczniki==21)
    {
         fuzzy_left=0;
         fuzzy_front=40;
         fuzzy_right=0;

    }
    if(liczniki==22)
    {
         fuzzy_left=1;
         fuzzy_front=40;
         fuzzy_right=0;

    }
    if(liczniki==23)
    {
         fuzzy_left=0;
         fuzzy_front=40;
         fuzzy_right=1;

    }
    if(liczniki==24)
    {
         fuzzy_left=1;
         fuzzy_front=40;
         fuzzy_right=1;

    }
    if(liczniki==25)
    {
         fuzzy_left=0;
         fuzzy_front=0;
         fuzzy_right=40;

    }
    if(liczniki==26)
    {
         fuzzy_left=1;
         fuzzy_front=0;
         fuzzy_right=40;

    }
    if(liczniki==27)
    {
         fuzzy_left=0;
         fuzzy_front=1;
         fuzzy_right=40;

    }
    if(liczniki==28)
    {
         fuzzy_left=1;
         fuzzy_front=1;
         fuzzy_right=40;

    }
    if(liczniki==29)
    {
         fuzzy_left=21;
         fuzzy_front=39;
         fuzzy_right=27;

    }
    */
bool ok;


  //  int left=100;
  //   int front=99;
  //  int right=100;




    int licznik_left=0,licznik_front=0,licznik_right=0;

    for(int i=0; i<=68921;i=i+1681)
    {

        int porowanie=tab_fuzzy[i].section(' ', 0, 0).toDouble(&ok);
        //int porowanie=tab_left[i].toDouble(&ok);
       // qDebug() << "tab_left1 " <<tab_left[i] << " por " <<porowanie;
       // qDebug() << "tab_left2 " <<tab_left[i].toDouble(&ok) << " por " <<porowanie;
        if(porowanie<=fuzzy_left)
        {
            licznik_left=licznik_left+1681;
            if(licznik_left>=68921)
                licznik_left=68921;

            if(fuzzy_left>=40 && fuzzy_front>=40 && fuzzy_right>=40)
            {
                licznik_left=68920;
                //qDebug() << "lalalaaaaa ";
            }
        }
        else
        {
            break;
        }

    }

    for(int i=licznik_left-1; i>=0;i--)
    {

        int porowanie=tab_fuzzy[i].section(' ', 1, 1).toDouble(&ok);
        //int porowanie=tab_front[i].toDouble(&ok);
         //qDebug() << "tab_front2 " <<tab_front[i].toDouble(&ok) << " por " <<porowanie;

        if(porowanie>fuzzy_front)
        {
            licznik_front++;
        }
        else
        {
           // licznik_front=licznik_front+80;
            break;

        }

    }



    if(fuzzy_left>=40 && fuzzy_front>=40 && fuzzy_right<=39)
    {
        //licznik_right=licznik_right;//+fuzzy_right;
   //     qDebug() << "bbbbbb ";
        licznik_front=41;
    }




    for(int i=licznik_front-1; i>=0;i--)
    {
        //qDebug() << "aaaaaaa ";


        int porowanie=tab_fuzzy[i].section(' ', 2, 2).toDouble(&ok);
        //int porowanie=tab_right[i].toDouble(&ok);
        if(porowanie>=fuzzy_right)
        {
            qDebug() << "aaaaaaa1 " ;
            licznik_right++;
        }
        else
        {
            break;
        }
    }



    if(fuzzy_left>=40 && fuzzy_front>=40 && fuzzy_right<=39)
    {
        //licznik_right=licznik_right;//+fuzzy_right;
    //   qDebug() << "bbbbbb7 ";
        licznik_front=0;
    }


    if(licznik_right > 40 )
        licznik_right=41-fuzzy_right;
    if(fuzzy_front == 40 && fuzzy_left !=40 && fuzzy_right !=40  )
        licznik_right=41-fuzzy_right;
    if(fuzzy_front == 40 && fuzzy_left !=40 && fuzzy_right ==40  )
        licznik_right=41-fuzzy_right;
    //int wynik=licznik_left-licznik_front-licznik_right+81;
    int wynik=licznik_left-licznik_front-licznik_right;

   lewy_silnik=tab_fuzzy[wynik].section(' ', 3, 3);
   prawy_silnik=tab_fuzzy[wynik].section(' ', 4, 4);
//lewy_silnik=tab_left_engine[wynik];
//prawy_silnik=tab_right_engine[wynik];
//    qDebug() << "fuzzy_left " <<fuzzy_left << "fuzzy_front " <<fuzzy_front << "fuzzy_right "<< fuzzy_right;
//    qDebug() << "licznik left " <<licznik_left ;
 //   qDebug() << "licznik front " <<licznik_front ;
 //   qDebug() << "licznik right " <<licznik_right ;
//    qDebug() << "wynik " <<wynik ;
 //   qDebug() << "lewy silnik " <<lewy_silnik << " prawy silnik " << prawy_silnik;
   // uaktualnij_dane(""+lewy_silnik+","+prawy_silnik);

    uaktualnij_dane("11,"+lewy_silnik+"*"+prawy_silnik);
   // liczniki++;
//    if (licznik_omijanie_przeszkod==10)
//    uaktualnij_dane("13,1");
// licznik_omijanie_przeszkod++;
}


//int liczniki=0;
void MainWindow::fuzylogic_cel()
{




   // kSetEncoders(ref,0,0);
  //  while(1)
  //  {
        //poz_enkoder_left=10;  // odczytanie pozycji enkodera lewego
        //poz_enkoder_right=10;
        cl=poz_enkoder_left; // ustawienie pozycja enkodera lewego
        cr=poz_enkoder_right  ;
        if ((cl_old != cl) | (cr_old != cr))  // jeśli został wykoanny ruch
        {
                dl = cl - cl_old;
                dr = cr - cr_old;
                cl_old = cl;
                cr_old = cr;

                if (dl != dr) // jeśli koła obróćiły się o różną odległość
                {
                          angle = (dr - dl)/DR*MPP; // kąt obrotu robota
                          radius = (DR/2)*(dl + dr)/(dr - dl); // odległość promienia skrętu R (ICC) od śrdoka robota
                          forward = radius*sin(angle); // jazda na przód ale w tym wypadku po łuku
                          lateral = radius*(1.0 - cos(angle)); // jazda w bok
                }
                else
                {

                          angle = 0.0; // jeżeli obroty kół były takie same to kąt 0
                          forward = dl*MPP; // jazda na przód
                          lateral = 0.0; // jazda w bok
                }

                        dx = cos(kat_theta);
                        dy = sin(kat_theta);
                        x = x + forward*dx - lateral*dy; //  obliczanie pozycji x
                        y = y + forward*dy + lateral*dx; //  obliczanie pozycji y
                        kat_theta =(kat_theta+ angle);   // zmiana pozycji kąta

           }
                            if (kat_theta >= 2*pi)  // jeżeli kąt większy niż 2pi to odejmnij 2pi
                            {
                             kat_theta =kat_theta - 2*pi;
                            }


                        if (kat_theta <= -2*pi)
                        {
                         kat_theta = kat_theta+2*pi; // jeżeli kąt większy niż 2pi to dodaj 2pi
                        }

                    psi=kat_theta-(atan2((yg-y),(xg-x))); // oblicz obrót od celu
                    z=sqrt(pow((xg-x),2)+pow((yg-y),2)); // oblicz odległość od celu
                    if (z<=5)   // jeśli odległość mniejsza niż 60 to zatrzymaj robota

                    {
                   // ksetspeed(ref,0,0);
                   // kSetEncoders(ref,0,0);
                   //     lewy_silnik= QString("%1").arg(0);
                   //     prawy_silnik= QString("%1").arg(0);
                        uaktualnij_dane("5");
                       // uaktualnij_dane("7");
                        finish=1;
                    //    poz_enkoder_left=0;
                   //     poz_enkoder_right=0;
                //    break;
                    }


                    bool ok;


/*


                    if(liczniki==0)
                    {
                        z=0.0;
                        psi=-6.3;

                    }
                    if(liczniki==1)
                    {
                        z=0;
                        psi=0;

                    }
                    if(liczniki==2)
                    {
                        z=0;
                        psi=6.3;

                    }
                    if(liczniki==3)
                    {
                        z=460;
                        psi=0;

                    }
                    if(liczniki==4)
                    {
                        z=460;
                        psi=-6.3;

                    }
                    if(liczniki==5)
                    {
                        z=460;
                        psi=6.3;
                    }
                    if(liczniki==6)
                    {
                        z=399;
                        psi=2;

                    }
                    if(liczniki==7)
                    {
                        z=399;
                        psi=-2;

                    }

                    if(liczniki==8)
                    {
                        z=1;
                        psi=-1;

                    }
                    if(liczniki==9)
                    {
                        z=1;
                        psi=0;

                    }
                    if(liczniki==10)
                    {
                      z=1;
                        psi=1;

                    }

*/



int licznik_z=0,licznik_psi=0;



                        for(int i=0; i<=74220;i=i+1)
                        {

                            int porowanie=tab_fuzzy_cel[i].section(' ', 0, 0).toDouble(&ok);
                            //int porowanie=tab_left[i].toDouble(&ok);
                           // qDebug() << "tab_left1 " <<tab_left[i] << " por " <<porowanie;
                           // qDebug() << "tab_left2 " <<tab_left[i].toDouble(&ok) << " por " <<porowanie;
                            if(porowanie<=z)
                            {
                                licznik_z=licznik_z+1;
                                if(licznik_z>=74220)
                                    licznik_z=74220;

                                if(z>=460 && psi>=4.0 )
                                {
                                    licznik_z=74220;
                                    //qDebug() << "lalalaaaaa ";
                                }
                            }
                            else
                            {
                                break;
                            }

                        }

                        for(int i=licznik_z-1; i>=0;i--)
                        {

                            double porowanie=tab_fuzzy_cel[i].section(' ', 1, 1).toDouble(&ok);
                            //int porowanie=tab_front[i].toDouble(&ok);
                           //  qDebug() << "tab_front2 " <<tab_fuzzy_cel[i].toDouble(&ok) << " por " <<porowanie;

                            if(porowanie>psi)
                            {
                                licznik_psi++;
                            }
                            else
                            {
                               // licznik_front=licznik_front+80;
                                break;

                            }

                        }

licznik_psi++;


                        //int wynik=licznik_left-licznik_front-licznik_right+81;
                        int wynik=licznik_z-licznik_psi;





                    lewy_silnik=tab_fuzzy_cel[wynik].section(' ', 2, 2);
                    prawy_silnik=tab_fuzzy_cel[wynik].section(' ', 3, 3);
                   // if(finish==0)

                   qDebug() << "wynik " <<wynik ;
                   qDebug() << " Z " <<z << " psi  " << psi*180/pi << " rad " << psi;
                   qDebug() << "lewy silnik " <<lewy_silnik << " prawy silnik " << prawy_silnik;
                   qDebug() << " z " <<z << "   psi  " <<psi ;

                   if(flaga_jazda_do_celu==true)
                        uaktualnij_dane("8,"+lewy_silnik+"*"+prawy_silnik+"*"+QString::number(z));
                   if(flaga_omijanie_przeszkod_jazda_do_celu==true)
                        uaktualnij_dane("16,"+lewy_silnik+"*"+prawy_silnik+"*"+QString::number(z));
                   if(flaga_planowanie_jazda_do_celu_fuzzy==true)
                        uaktualnij_dane("18,"+lewy_silnik+"*"+prawy_silnik+"*"+QString::number(z));



                   // v = evalfis([z, psi], K);
                    //ksetspeed(ref,v(1),v(2));

                 //   qDebug() << "licznik z " <<licznik_z ;
                 //   qDebug() << "licznik psi " <<licznik_psi ;

                    // [z psi*180/pi]


  //  }


}



void MainWindow::fuzylogic_jedz_do_celu_po_prostej()
{




   // kSetEncoders(ref,0,0);
  //  while(1)
  //  {
        //poz_enkoder_left=10;  // odczytanie pozycji enkodera lewego
        //poz_enkoder_right=10;
        cl=poz_enkoder_left; // ustawienie pozycja enkodera lewego
        cr=poz_enkoder_right;
        if ((cl_old != cl) | (cr_old != cr))  // jeśli został wykoanny ruch
        {
                dl = cl - cl_old;
                dr = cr - cr_old;
                cl_old = cl;
                cr_old = cr;

                if (dl != dr) // jeśli koła obróćiły się o różną odległość
                {
                          angle = (dr - dl)/DR*MPP; // kąt obrotu robota
                          radius = (DR/2)*(dl + dr)/(dr - dl); // odległość promienia skrętu R (ICC) od śrdoka robota
                          forward = radius*sin(angle); // jazda na przód ale w tym wypadku po łuku
                          lateral = radius*(1.0 - cos(angle)); // jazda w bok
                }
                else
                {

                          angle = 0.0; // jeżeli obroty kół były takie same to kąt 0
                          forward = dl*MPP; // jazda na przód
                          lateral = 0.0; // jazda w bok
                }

                        dx = cos(kat_theta);
                        dy = sin(kat_theta);
                        x = x + forward*dx - lateral*dy; //  obliczanie pozycji x
                        y = y + forward*dy + lateral*dx; //  obliczanie pozycji y
                        kat_theta =(kat_theta+ angle);   // zmiana pozycji kąta

           }
                            if (kat_theta >= 2*pi)  // jeżeli kąt większy niż 2pi to odejmnij 2pi
                            {
                             kat_theta =kat_theta - 2*pi;
                            }


                        if (kat_theta <= -2*pi)
                        {
                         kat_theta = kat_theta+2*pi; // jeżeli kąt większy niż 2pi to dodaj 2pi
                        }

                    psi=kat_theta-(atan2((yg-y),(xg-x))); // oblicz obrót od celu
                    z=sqrt(pow((xg-x),2)+pow((yg-y),2)); // oblicz odległość od celu
                    if (z<=10)   // jeśli odległość mniejsza niż 60 to zatrzymaj robota

                    {
                   // ksetspeed(ref,0,0);
                   // kSetEncoders(ref,0,0);
                   //     lewy_silnik= QString("%1").arg(0);
                   //     prawy_silnik= QString("%1").arg(0);
                        uaktualnij_dane("5,1");
                       // uaktualnij_dane("7");
                        finish=1;
                    //    poz_enkoder_left=0;
                   //     poz_enkoder_right=0;
                //    break;
                    }





                    bool ok;


                      //  int left=100;
                      //   int front=99;
                      //  int right=100;




                        int licznik_z=0,licznik_psi=0;

                        for(int i=0; i<=321200;i=i+1)
                        {

                            int porowanie=tab_fuzzy_cel[i].section(' ', 0, 0).toDouble(&ok);
                            //int porowanie=tab_left[i].toDouble(&ok);
                           // qDebug() << "tab_left1 " <<tab_left[i] << " por " <<porowanie;
                           // qDebug() << "tab_left2 " <<tab_left[i].toDouble(&ok) << " por " <<porowanie;
                            if(porowanie<=z)
                            {
                                licznik_z=licznik_z+1;
                                if(licznik_z>=321200)
                                    licznik_z=321200;

                                if(z>=400 && psi>=4 )
                                {
                                    licznik_z=321199;
                                    //qDebug() << "lalalaaaaa ";
                                }
                            }
                            else
                            {
                                break;
                            }

                        }

                        for(int i=licznik_z-1; i>=0;i--)
                        {

                            double porowanie=tab_fuzzy_cel[i].section(' ', 1, 1).toDouble(&ok);
                            //int porowanie=tab_front[i].toDouble(&ok);
                           //  qDebug() << "tab_front2 " <<tab_fuzzy_cel[i].toDouble(&ok) << " por " <<porowanie;

                            if(porowanie>psi)
                            {
                                licznik_psi++;
                            }
                            else
                            {
                               // licznik_front=licznik_front+80;
                                break;

                            }

                        }

licznik_psi++;


                        //int wynik=licznik_left-licznik_front-licznik_right+81;
                        int wynik=licznik_z-licznik_psi;





                    lewy_silnik=tab_fuzzy_cel[wynik].section(' ', 2, 2);
                    prawy_silnik=tab_fuzzy_cel[wynik].section(' ', 3, 3);
                    if(finish==0)

                   // v = evalfis([z, psi], K);
                    //ksetspeed(ref,v(1),v(2));

                 //   qDebug() << "licznik z " <<licznik_z ;
                 //   qDebug() << "licznik psi " <<licznik_psi ;
                    qDebug() << "wynik " <<wynik ;
                   qDebug() << " Z " <<z << " psi  " << psi*180/pi << " rad " << psi;
                   qDebug() << "lewy silnik " <<lewy_silnik << " prawy silnik " << prawy_silnik;

                 uaktualnij_dane("14,"+lewy_silnik+"*"+prawy_silnik+"*"+QString::number(z));
                    // [z psi*180/pi]


  //  }


}


void MainWindow::Readfile(QString filename)
{

}

void MainWindow::on_pushButton_10_clicked()
{

//finish=0;
//fuzylogic_jedz();
//fuzylogic_cel();


    int xx[1000],yy[1000],licznik=0;
    int z1,z2,z3,z4,licznik_z=0;
    int xxx[1000],yyy[1000],licznik_xxx=1;
    int  dlugosc_odcinka[1000], kat_nachylenia,kat_nachylenia_old[1000];
    double wsp, Y, X;
    //z=6;

    flaga_omijanie_przeszkod=false;
    flaga_jazda_do_celu=false;
    flaga_omijanie_przeszkod_jazda_do_celu=false;
    flaga_planowanie_jazda_do_celu_fuzzy=true;

/*    for(int i=0; i<1000; i=i+1)
    {
        xx[i]=0;
        yy[i]=0;
    }
*/

            //licznik_traj_robot;
    //    traj_robot[i].x_pos=x;
    //    traj_robot[i].y_pos=y;

    if(ui->checkBox_4->isChecked())
    {
// wersja dopbra wybieranie co ile z menu
for(int i=0; i<=licznik_traj_robot; i=i+ui->spinBox_9->value())
{
 xx[licznik]=traj_robot[i].x_pos;
 yy[licznik]=traj_robot[i].y_pos;
rysujPrzeszkode( xx[licznik], yy[licznik], 5, 5,color_check_point);
qDebug() << "xx  " <<xx[licznik]  << " yy  " << yy[licznik] ;
licznik++;
}
licznik--;
xx[licznik]=met.x_pos;
yy[licznik]=met.y_pos;


    for(int i=0; i<=licznik; i=i+1)
    {

       // xg=xx[i];
       // yg=-yy[i];


         xg_planowanie[i]=xx[i];
         yg_planowanie[i]=-yy[i];
         licznik_xg_yg++;

        qDebug() << "xx  " << xx[i] << " yy " << yy[i] ;
        qDebug() << "qaz  " << i << " " << z << " xg " << xg << " yg " << yg;
       // do
         //   {
                qDebug() << "zzz  " << i << " " << z << " xg " << xg << " yg " << yg;

           // }while(z>5);

    }

     xg=xg_planowanie[0];
     yg=yg_planowanie[0];
licznik_xg_yg--;
     for(int i=0; i<=licznik_xg_yg; i=i+1)
     {
         qDebug() << "xg_planowanie  " << xg_planowanie[i] << " yg_planowanie " << yg_planowanie[i] << " " << i ;
     }


    fuzylogic_cel();
    }

    if(ui->checkBox_5->isChecked() || ui->checkBox_6->isChecked())
    {

for(int i=0; i<=licznik_traj_robot; i=i+1)
{
    z1=traj_robot[i].x_pos;
    z2=traj_robot[i+10].x_pos;
    z3=traj_robot[i].y_pos;
    z4=traj_robot[i+10].y_pos;
 if(abs(z2-z1)>1 && abs(z3-z4)>1 )
 {
 xx[licznik_z]=traj_robot[i].x_pos;
 yy[licznik_z]=traj_robot[i].y_pos;
 qDebug() << "xx1  " <<xx[licznik_z]  << " yy1  " << yy[licznik_z] << " licznik_traj_robot  " << licznik_traj_robot ;;
 licznik_z++;

 }
}

for(int i=0; i<licznik_z; i=i+10)
{
    z1=xx[i]+yy[i];
    for(int j=i; j<=i+10; j=j+1)
    {
    z2=xx[j]+yy[j];

    if(abs(z2-z1)>20 )//if(abs(z2-z1)>30 )
    {
    xxx[licznik_xxx]=xx[i];
    yyy[licznik_xxx]=yy[i];
    licznik_xxx++;
    xxx[licznik_xxx]=xx[j];
    yyy[licznik_xxx]=yy[j];
    qDebug() << "xx2  " <<xxx[licznik_xxx]  << " yy2  " << yyy[licznik_xxx] << " licznik_z  " << licznik_z ;
    licznik_xxx++;



    break;
    }

    }

}

xxx[0]=start_robota.x_pos;
yyy[0]=start_robota.y_pos;
xxx[licznik_xxx-1]=met.x_pos;
yyy[licznik_xxx-1]=met.y_pos;
//licznik_xxx++;
for(int i=0; i<licznik_xxx; i=i+1)
{
rysujPrzeszkode( xxx[i], yyy[i], 5, 5,color_check_point);
qDebug() << "xx3  " <<xxx[i]  << " yy3  " << yyy[i] << " licznik_xxx  " << licznik_xxx ;
}




if(ui->checkBox_6->isChecked()==false)
{
    for(int i=0; i<licznik_xxx; i=i+1)
    {
      //  xg=xxx[i];
      //  yg=-yyy[i];

        xg_planowanie[i]=xxx[i];
        yg_planowanie[i]=-yyy[i];
        licznik_xg_yg++;
      //  qDebug() << "xx  " << xxx[i] << " yy " << yyy[i] ;
      //  qDebug() << "qaz  " << i << " " << z << " xg " << xg << " yg " << yg;
       // do
        //    {
             //   qDebug() << "zzz  " << i << " " << z << " xg " << xg << " yg " << yg;

           // }while(z>5);

    }

    xg=xg_planowanie[0];
    yg=yg_planowanie[0];
licznik_xg_yg--;
    for(int i=0; i<=licznik_xg_yg; i=i+1)
    {
        qDebug() << "xg_planowanie  " << xg_planowanie[i] << " yg_planowanie " << yg_planowanie[i] << " " << i ;
    }

    fuzylogic_cel();
}





    }



    if(ui->checkBox_6->isChecked())
    {

        QString dane_mapy;
        dane_mapy="15,";
        dane_mapy+=QString::number(licznik_xxx-1);
        dane_mapy+="*";


        for(int i=1; i<licznik_xxx; i=i+1)
        {
         Y= yyy[i]-yyy[i-1];
         X= xxx[i]-xxx[i-1];
         wsp = (Y/X);
         kat_nachylenia=qAtan(wsp)*180/pi;
         dlugosc_odcinka[i] = sqrt(X*X + Y*Y);
         kat_nachylenia_old[i]=kat_nachylenia;
         qDebug() << "Y  " << Y;
         qDebug() << "X  " << X;
        qDebug() << "współczynnik kierunkowy  " << wsp;
        qDebug() << "kat nachylenia  " << kat_nachylenia;
        qDebug() << "dlugosc odcinka  " << dlugosc_odcinka[i] ;
        }

        for(int i=1; i<licznik_xxx; i=i+1)
        {
            if(i>1 &&  kat_nachylenia_old[i-1]>= 0 &&  kat_nachylenia_old[i]>= 0 )
            {
                kat_nachylenia=kat_nachylenia_old[i-1]-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i>1 &&  kat_nachylenia_old[i-1]>= 0 &&  kat_nachylenia_old[i]< 0)
            {
                kat_nachylenia=-kat_nachylenia_old[i]-kat_nachylenia_old[i-1];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i>1 &&  kat_nachylenia_old[i-1]< 0 &&  kat_nachylenia_old[i]>= 0)
            {
                kat_nachylenia=-kat_nachylenia_old[i-1]-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i>1 &&  kat_nachylenia_old[i-1]< 0 &&  kat_nachylenia_old[i]< 0)
            {
                kat_nachylenia=kat_nachylenia_old[i-1]-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i==1)
            {
                kat_nachylenia=-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }

        dane_mapy+="*";
        dane_mapy+=QString::number(dlugosc_odcinka[i]);
        dane_mapy+="*";

        }

       dane_mapy=dane_mapy.left(dane_mapy.length()-1);
       qDebug() << "1aasdasd:  " << dane_mapy ;
       uaktualnij_dane(dane_mapy);


    }


 //   flaga_omijanie_przeszkod_jazda_do_celu=false;
  //  fuzylogic_cel();








}




void MainWindow::on_pushButton_7_clicked()
{
   // fuzylogic();


  //  extern int map[n][m];








    for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++) mapp[x][y]=0;
        }


    for(int i=0;i<n;i++)
           {
            for(int j=0;j<m;j++)
               mapp[i][j]=0;
           }
    for(int i=0;i<n;i++)
           {
            for(int j=0;j<m;j++)
               mapp[i][j]=0;
           }

    for(int i=0;i<n;i++)
           {
            for(int j=0;j<m;j++)
            {
               mapp[i][j]=moja_mapa[i][j];
            //   qDebug() << "moja_mapa " <<moja_mapa[i][j] ;
            }
           }

        // randomly select start and finish locations




int counter=0;
      int xA[n], yA[n], xB[n], yB[n];

      for(int i=0;i<n;i++)
             {
             xA[i]=i;
             yA[i]=i;
             xB[i]=i;
             yB[i]= i;

             }

 //   xA=1; yA=1; //xB=150; yB=180;
 //  int aa =met.x_pos;
  //  int bb =met.y_pos;
 //    xB=aa; yB=bb;
        cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
        cout<<"Start: "<<xA[0]<<","<<yA[0]<<endl;
        cout<<"Finish: "<<xB[met.x_pos]<<","<<yB[met.y_pos]<<endl;
        // get the route
        clock_t start = clock();
        string route=pathFind(xA[0], yA[0], xB[met.x_pos], yB[met.y_pos]);
        if(route=="") cout<<"An empty route generated!"<<endl;
        clock_t end = clock();
        double time_elapsed = double(end - start);
        cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
        cout<<"Route:"<<endl;

         QString sciezka(route.c_str());
          QString traj;
        qDebug()<<sciezka;
ui->plainTextEdit->appendHtml(sciezka);

int old_x=2, old_y=2;
        // follow the route on the map and display it
           if(route.length()>0)
           {
               int j; char c;
               int x=xA[0];
               int y=yA[0];
               mapp[x][y]=2;
               for(int i=0;i<route.length();i++)
               {
                   c =route.at(i);
                   j=atoi(&c);
                   x=x+dxx[j];
                   y=y+dyy[j];
                   mapp[x][y]=3;
                   traj = QString("xx: %1 yx: %2 ").arg(x).arg(y);
                  ui->plainTextEdit->appendHtml(traj);
                  traj_robot[i].x_pos=x;
                  traj_robot[i].y_pos=y;
                  licznik_traj_robot++;
                  rysujTraseRobot2(x,y);
               }
               mapp[x][y]=4;

               // display the map with the route
               for(int y=0;y<m;y++)
               {
                   for(int x=0;x<n;x++)
                       if(mapp[x][y]==0)
                          cout<<".";
                       else if(mapp[x][y]==1)
                           cout<<"O"; //obstacle
                       else if(mapp[x][y]==2)
                           cout<<"S"; //start
                       else if(mapp[x][y]==3)
                       //if(map[x][y]==3)
                       {
                           cout<<"R"; //route
                      //     traj = QString("xx: %1 yx: %2 ").arg(x).arg(y);
                      //    ui->plainTextEdit->appendHtml(traj);
                           //cout<< " x: "<<x << "y: "<<  y  << " counter " << counter; //route
                  //if(old_x==x && old_y == y || old_x==x+1 && old_y == y || old_x==x-1 && old_y == y || old_x==x && old_y == y-1 || old_x==x && old_y == y+1 || old_x==x+1 && old_y == y+1 || old_x==x-1 && old_y == y-1 || old_x==x && old_y == y+1)
                // if(old_x<=x+10 && old_y<=y+10  && old_x>=x-10 && old_y>=y-10  || old_x>=x+10 && old_y >= y+10 )
                          if(mapp[x][y]!=1)
                          {
                         //  rysujTraseRobot2(x,y);

                //    traj = QString("x: %1 y: %2 ").arg(x).arg(y);
                //   ui->plainTextEdit->appendHtml(traj);
                   counter++;
                   old_x=x;
                   old_y=y;
                  }
                       }
                       else if(mapp[x][y]==4)
                           cout<<"F"; //finish
                   cout<<endl;
               }
           }




    rysujPrzeszkode( met.x_pos, met.y_pos, 10, 10,color_meta);




}



void MainWindow::rysujTraseRobot2(int x1, int y1)
{

    if(x1<=width && y1 <= height && x1>0 && y1>0)
    {

        //        p1 = ui->graphicsView->mapFromGlobal(event->globalPos());
                p1r=lastpoint_robot2;

//                p2 = ui->graphicsView->mapFromGlobal(event->globalPos());

                int k=1;
                k=ui->spinBox_8->value();
                int xx1=p1r.x();
                int yy1=p1r.y();
             //   qDebug() << "2aap2:   " <<xx1;
              //  qDebug() << "2aap2:   " <<yy1;
                p1r.setX(p1r.x()-xx1%k);
                p1r.setY(p1r.y()-yy1%k);
             //   qDebug() << "3aap2:   " <<p1r.x();
             //   qDebug() << "3aap2:   " <<p1r.y();

                  p2r.setY(y1);
                  p2r.setX(x1);


                lastpoint_robot2=p2r;

qDebug() << "3a:" ;

                drawLineRobot2();
            //    reset();
             //   scene->addPixmap(QPixmap::fromImage(img));




        }

}


void MainWindow::drawLineRobot2()
{

drawCross(colorlline);
    int pom=0;
    //I assume we have a coordinate system with the origin in the upper left corner
    //with the positive axes down and to the right, this is set up in the ctor



    int k=1;
    k=ui->spinBox_3->value();


   // qDebug() << "1aap2:   " <<k;
    int xx=p2r.x();
    int yy=p2r.y();
  //  qDebug() << "2aap2:   " <<xx;
  //  qDebug() << "2aap2:   " <<yy;
    p2r.setX(p2r.x()-xx%k);
    p2r.setY(p2r.y()-yy%k);
  //  qDebug() << "3aap2:   " <<p2r.x();
  //  qDebug() << "3aap2:   " <<p2r.y();



    //first point
    QPoint f = p1r.toPoint();
    //last point
    QPoint l = p2r.toPoint();
//qDebug() << "4:" ;
    //vertical line
    if(f.x() == l.x())
    {
     //   qDebug() << "vertical line";

        if(f.y() > l.y())
        {
            qSwap(f, l);
        }

        for(int y=f.y(); y<=l.y(); y++)
        {

            if(pom%ui->spinBox->value()==0)
            {
             //  qDebug() << "x:" << f.x() << "y:" << y;
              // QString tmp = QString("%1,%2,%3,%4").arg("x").arg(f.x()).arg("y").arg(y);
               //uaktualnij_dane(tmp);
              // Sleeper::usleep(10);
              // Sleeper::msleep(100);
              // Sleeper::sleep(10);

            }
            pom++;
            img.setPixel(f.x(), y, color_robot_black);
            img.setPixel(f.x()+1, y+1, color_robot_black);
            img.setPixel(f.x()-1, y-1, color_robot_black);
        }

        return;
    }

    //slope
    qreal m = (p2r.y() - p1r.y())/(p2r.x() - p1r.x());
   // qDebug() << "Slope:" << m;

    //in order to get better effects
    //I should render the line iterating through the longer side
    //the if condition translates into: the rise is higher than the run
    if(m > 1.0 || m < -1.0)
    {
     //   qDebug() << "the rise is higher than the run";

        m = 1/m;

        //draw the line from the top to bottom
        if(f.y() > l.y())
        {
            qSwap(f, l);
        }

        for(int y=f.y(); y<=l.y(); y++)
        {
            int x = round(m*(y-f.y()) + f.x());

            if(pom%ui->spinBox->value()==0)
            {
             //  qDebug() << "x:" << x << "y:" << y;
             //  QString tmp = QString("%1,%2,%3,%4").arg("x").arg(x).arg("y").arg(y);
             //  uaktualnij_dane(tmp);
              // Sleeper::msleep(100);
            }
            pom++;

            img.setPixel(x, y, color_robot_black);
            img.setPixel(x+1, y+1, color_robot_black);
            img.setPixel(x-1, y-1, color_robot_black);
        }
    }
    else{
        //if the line was drawn right to left, make it so I draw left to right
 if(f.x() > l.x())
        {
            qSwap(f, l);
        }

        for(int x=f.x(); x<=l.x(); x++)
        {
            int y = round(m*(x-f.x()) + f.y());


            if(pom%ui->spinBox->value()==0)
            {
            //   qDebug() << "x:" << x << "y:" << y;
            //   QString tmp = QString("%1,%2,%3,%4").arg("x").arg(x).arg("y").arg(y);
            //   uaktualnij_dane(tmp);
             //  Sleeper::msleep(100);
            }
            pom++;


            img.setPixel(x, y,color_robot_black );
            img.setPixel(x+1, y+1,color_robot_black );
            img.setPixel(x-1, y-1,color_robot_black );

        }
    }

}





void MainWindow::rysujTraseRobot(int x1, int y1)
{

    if(x1<=width && y1 <= height && x1>0 && y1>0)
    {
        //        p1 = ui->graphicsView->mapFromGlobal(event->globalPos());
                p1r=lastpoint_robot;

//                p2 = ui->graphicsView->mapFromGlobal(event->globalPos());

                int k=1;
                k=ui->spinBox_8->value();
                int xx1=p1r.x();
                int yy1=p1r.y();
                qDebug() << "2aap2:   " <<xx1;
                qDebug() << "2aap2:   " <<yy1;
                p1r.setX(p1r.x()-xx1%k);
                p1r.setY(p1r.y()-yy1%k);
                qDebug() << "3aap2:   " <<p1r.x();
                qDebug() << "3aap2:   " <<p1r.y();

                  p2r.setY(y1);
                  p2r.setX(x1);


                lastpoint_robot=p2r;

qDebug() << "3a:" ;
                drawLineRobot();
                reset();
                scene->addPixmap(QPixmap::fromImage(img));

    }
}


void MainWindow::drawLineRobot()
{

drawCross(colorlline);
    int pom=0;
    //I assume we have a coordinate system with the origin in the upper left corner
    //with the positive axes down and to the right, this is set up in the ctor



    int k=1;
    k=ui->spinBox_3->value();


   // qDebug() << "1aap2:   " <<k;
    int xx=p2r.x();
    int yy=p2r.y();
  //  qDebug() << "2aap2:   " <<xx;
  //  qDebug() << "2aap2:   " <<yy;
    p2r.setX(p2r.x()-xx%k);
    p2r.setY(p2r.y()-yy%k);
  //  qDebug() << "3aap2:   " <<p2r.x();
  //  qDebug() << "3aap2:   " <<p2r.y();



    //first point
    QPoint f = p1r.toPoint();
    //last point
    QPoint l = p2r.toPoint();
qDebug() << "4:" ;
    //vertical line
    if(f.x() == l.x())
    {
     //   qDebug() << "vertical line";

        if(f.y() > l.y())
        {
            qSwap(f, l);
        }

        for(int y=f.y(); y<=l.y(); y++)
        {

            if(pom%ui->spinBox->value()==0)
            {
             //  qDebug() << "x:" << f.x() << "y:" << y;
               QString tmp = QString("%1,%2,%3,%4").arg("x").arg(f.x()).arg("y").arg(y);
               uaktualnij_dane(tmp);
              // Sleeper::usleep(10);
              // Sleeper::msleep(100);
              // Sleeper::sleep(10);

            }
            pom++;
            img.setPixel(f.x(), y, color_robot);
            img.setPixel(f.x()+1, y+1, color_robot);
            img.setPixel(f.x()-1, y-1, color_robot);
        }

        return;
    }

    //slope
    qreal m = (p2r.y() - p1r.y())/(p2r.x() - p1r.x());
   // qDebug() << "Slope:" << m;

    //in order to get better effects
    //I should render the line iterating through the longer side
    //the if condition translates into: the rise is higher than the run
    if(m > 1.0 || m < -1.0)
    {
        qDebug() << "the rise is higher than the run";

        m = 1/m;

        //draw the line from the top to bottom
        if(f.y() > l.y())
        {
            qSwap(f, l);
        }

        for(int y=f.y(); y<=l.y(); y++)
        {
            int x = round(m*(y-f.y()) + f.x());

            if(pom%ui->spinBox->value()==0)
            {
               qDebug() << "x:" << x << "y:" << y;
               QString tmp = QString("%1,%2,%3,%4").arg("x").arg(x).arg("y").arg(y);
               uaktualnij_dane(tmp);
              // Sleeper::msleep(100);
            }
            pom++;

            img.setPixel(x, y, color_robot);
            img.setPixel(x+1, y+1, color_robot);
            img.setPixel(x-1, y-1, color_robot);
        }
    }
    else{
        //if the line was drawn right to left, make it so I draw left to right
 if(f.x() > l.x())
        {
            qSwap(f, l);
        }

        for(int x=f.x(); x<=l.x(); x++)
        {
            int y = round(m*(x-f.x()) + f.y());


            if(pom%ui->spinBox->value()==0)
            {
               qDebug() << "x:" << x << "y:" << y;
               QString tmp = QString("%1,%2,%3,%4").arg("x").arg(x).arg("y").arg(y);
               uaktualnij_dane(tmp);
             //  Sleeper::msleep(100);
            }
            pom++;


            img.setPixel(x, y,color_robot );
            img.setPixel(x+1, y+1,color_robot );
            img.setPixel(x-1, y-1,color_robot );

        }
    }

}



void MainWindow::on_pushButton_12_clicked()
{
    if(ui->checkBox_12->isChecked())
    {
        uaktualnij_dane("6,100");

  /*      for(int i=0;i<50;i++)
        {
            rysujTraseRobot( i*2,  i);
        }
*/
    }
    if(ui->checkBox_13->isChecked())
    {
    uaktualnij_dane("7");
    }

}

void MainWindow::on_pushButton_8_clicked()
{
    //  scene->clear();
   //   drawCross(colorllinewhite);
   //   scene->clearFocus();
  //    scene->clearSelection();
 /*     for(int i=0;i<=460;i++)
      {
          for(int j=0;j<=350;j=j+1)
          {
          img.setPixel(i, j,color );

          }

      }
*/


    for(int i=0;i<licznik_xg_yg;i++)
    {

    xg_planowanie[i]=0;
    yg_planowanie[i]=0;

    }
    licznik_xg_yg=0;


    for(int i=0;i<licznik_traj_robot;i++)
    {

    traj_robot[i].x_pos =0;
    traj_robot[i].y_pos =0;
    }
licznik_traj_robot=0;

    for(int i=0;i<licznik_jazdy_po_prostej;i++)
    {
     kierunek_jazdy_po_prostej[i] =0;
     odleglosc_jazdy_po_prostej[i] =0;
    }
    licznik_jazdy_po_prostej=0;



rysujTraseRobot2(1,1);
scene->addPixmap(QPixmap::fromImage(img));
scene->backgroundBrush();
reset();
img.fill(Qt::white);

for(int i=0;i<n;i++)
       {
        for(int j=0;j<m;j++)
           moja_mapa[i][j]=0;
       }
//extern int map[n][m];
for(int i=0;i<n;i++)
       {
        for(int j=0;j<m;j++)
           mapp[i][j]=0;
       }
  //    scene = new QGraphicsScene(this);
   //   ui->graphicsView->setScene(scene);





 poz_x_jada_do_celu_po_prostej=0;
 poz_y_jada_do_celu_po_prostej=0;
 poz_x_jada_do_celu_po_prostej_old=0;
 poz_y_jada_do_celu_po_prostej_old=0;
 licznik_jazdy_po_prostej=0;
 kierunek_jazdy_robota=0;
 kierunek_jazdy_robota_old=0;
 licznik_planowanie=0;
 licznik_xg_yg=0;

  flaga_omijanie_przeszkod=false;
  flaga_jazda_do_celu=false;
  flaga_omijanie_przeszkod_jazda_do_celu=false;
  flaga_planowanie_jazda_do_celu_fuzzy=false;
  flaga_linefallower=false;
  licznik_omijanie_przeszkod=0;


   finish=0;
   x=0; // punkty na osi x w lokalnym układzie współrzędnych
   y=0; // punkty na osi x w lokalnym układzie współrzędnych
   xg=100; // pozycja x w globalnym układzie współrzędnych robota cel
   yg=0; // pozycja y w globalnym układzie współrzędnych robota cel
   poz_enkoder_left=0; // pozycja enkodera z silnika lewego
   poz_enkoder_right=0; // pozycja enkodera z silnika lewego
   DR=21.0;   // średnica robota w mm
  //  double DR=21.4;  od 19 do 23   // średnica robota w mm
   MPP=0.0224; //0.0345;  // promień koła w cm  lub *0.1 cm  ponoć◙ kephera ma r=8mm a piasło 0.08
  // 2 * 3.14 * 3.45 = 21,5    // 960 *0.02 = 19.2   //
   kat_theta=0; //  orientacja robota w globalnym układzie współrzędnych
   cl=0;// pozycja enkodera lewego
   cr=0;// pozycja enkodera prawego
   cl_old=0;// stara pozycja enkodera lewego
   cr_old=0;// stara pozycja enkodera prawego
   z=0; // odległość od celu
  //double i=0;
   psi=0; // kąt obrotu od celu jeśli 0 to na wprost celu
   pi=3.14159265359;
   dl=0;
   dr=0; // różnica o jaką obróciło się około
   angle=0;
   radius=0;
   forward=0;
   lateral=0;
   dx=0;
   dy=0;
   numer_przeszkody=0;

   start_robota.x_pos=0;
   start_robota.y_pos=0;

   lastpoint_robot.setX(1);
   lastpoint_robot.setY(1);
   lastpoint_robot2.setX(1);
   lastpoint_robot2.setY(1);

   kierunek_jazdy_po_prostej[0]=0;
   odleglosc_jazdy_po_prostej[0]=0;

   flaga_rysowanie_przeszkod=false;

}

void MainWindow::on_pushButton_13_clicked()
{




    int xx,yy;
    xx=ui->spinBox_4->value();
    yy=ui->spinBox_5->value();
met.x_pos=xx;
met.y_pos=yy;


//int xx,yy;
//xx=ui->spinBox_4->value();
//yy=ui->spinBox_5->value();
xg=xx;
yg=-yy;





/*
for(int i=0;i<n;i++)
       {
        for(int j=0;j<m;j++)
           moja_mapa[i][j]=0;
       }
extern int map[n][m];
for(int i=0;i<n;i++)
       {
        for(int j=0;j<m;j++)
           map[i][j]=0;
       }
*/
on_pushButton_14_clicked();
//on_pushButton_8_clicked();
    rysujPrzeszkode( xx, yy, 10, 10,color_meta);
    on_pushButton_14_clicked();
//on_pushButton_8_clicked();


    if(flaga_rysowanie_przeszkod==true)
    {
        on_pushButton_18_clicked();
    }
}

void MainWindow::on_pushButton_14_clicked()
{
    rysujTraseRobot2(1,1);
    scene->addPixmap(QPixmap::fromImage(img));
    scene->backgroundBrush();
    reset();
   // img.fill(Qt::white);

    for(int i=0;i<n;i++)
           {
            for(int j=0;j<m;j++)
               moja_mapa[i][j]=0;
           }
  //  extern int map[n][m];
    for(int i=0;i<n;i++)
           {
            for(int j=0;j<m;j++)
               mapp[i][j]=0;
           }
}

/*
void MainWindow::on_pushButton_15_clicked()
{
    int xx,yy;
    xx=ui->spinBox_4->value();
    yy=ui->spinBox_5->value();
xg=xx;
yg=yy;



}
*/

void MainWindow::rysujRadar(int x1, int y1, int x2, int y2,int start_angle,  int end_angle, QColor color_radar)
{

   /*
    start_angle=start_angle*16;
end_angle=end_angle*16;


    QPen pen;
    pen.setColor(color_radar);
    pen.setWidth(1);
    QGraphicsRectItem *item2 = new QGraphicsEllipseItem(x1, y1, x2, y2);
    //QGraphicsEllipseItem* item2 = new QGraphicsEllipseItem(x1, y1, x2, y2);
    item2->setPen(pen);
    item2->setStartAngle(start_angle);
    item2->setSpanAngle(end_angle - start_angle);
    //item2->setBrush(QColor(255, 0, 0));
    item2->setBrush(color_radar);
    scene2->addItem(item2);

*/

    start_angle=start_angle*16;
end_angle=end_angle*16;

    pen.setColor(color_radar);
    paint->setPen(pen);

                    paint->setPen(pen);
                    paint->setBrush(color_radar);
                    paint->setBackgroundMode(Qt::OpaqueMode);
                 //   paint->drawPie(x1, y1, x2,y2, start_angle, end_angle-start_angle);

                    paint->setPen(pen);
                 //    paint->drawArc(x1, y1, x2,y2, start_angle, end_angle-start_angle);
                    // paint->drawChord(x1, y1, x2,y2, start_angle, end_angle);

                    // paint->drawLine(x1, y1, x2,y2);
                   //  paint->drawArc(x1, y1, x2,y2, start_angle, end_angle);
                    //paint->drawArc(100, 100, 100,50, 50, 90);

          //  paint->setPen(*(new QColor(255,34,255,255)));
         //   paint->drawRect(15,15,100,100);





  //  scene->addEllipse(100, 50, 50, 50, pen, QBrush(Qt::red) );



int width=340;
int height=460;
int iAngle=45;
int PI=M_PI;
int iDistance=20;

int TWO_PI=2*M_PI;
paint->translate(960,1000);

/*
            paint->drawArc(0, 0,(width-width*0.0625),(width-width*0.0625), 180, 360);
             paint->drawArc(0, 0,(width-width*0.27),(width-width*0.27), 180, 360);
              paint->drawArc(0, 0,(width-width*0.479),(width-width*0.479), 180, 360);
               paint->drawArc(0, 0,(width-width*0.687),(width-width*0.687), 180, 360);
             paint->drawLine(-width/2,0,width/2,0);
             paint->drawLine(0,0,(-width/2)*cos(30),(-width/2)*sin(30));
                    paint->drawLine(0,0,(-width/2)*cos((30)),(-width/2)*sin((30)));
                    paint->drawLine(0,0,(-width/2)*cos((60)),(-width/2)*sin((60)));
                    paint->drawLine(0,0,(-width/2)*cos((90)),(-width/2)*sin((90)));
                   paint->drawLine(0,0,(-width/2)*cos((120)),(-width/2)*sin((120)));
                    paint->drawLine(0,0,(-width/2)*cos((150)),(-width/2)*sin((150)));
                    paint->drawLine((-width/2)*cos((30)),0,width/2,0);

                  */


                    // draws the arc lines
                     paint->drawArc(0,0,1800,1800,PI,TWO_PI);
                     paint->drawArc(0,0,1400,1400,PI,TWO_PI);
                     paint->drawArc(0,0,1000,1000,PI,TWO_PI);
                     paint->drawArc(0,0,600,600,PI,TWO_PI);
                    // draws the angle lines
                    paint->drawLine(-960,0,960,0);

                    double radiany =( 30 * M_PI ) / 180.0f;
                    paint->drawLine(0,0,-960*cos((radiany)),-960*sin((radiany)));
                    radiany =( 60 * M_PI ) / 180.0f;
                    paint->drawLine(0,0,-960*cos((radiany)),-960*sin((radiany)));
                    radiany =( 90 * M_PI ) / 180.0f;
                    paint->drawLine(0,0,-960*cos((radiany)),-960*sin((radiany)));
                    radiany =( 120 * M_PI ) / 180.0f;
                    paint->drawLine(0,0,-960*cos((radiany)),-960*sin((radiany)));
                    radiany =( 150 * M_PI ) / 180.0f;
                    paint->drawLine(0,0,-960*cos((radiany)),-960*sin((radiany)));
                    radiany =( 30 * M_PI ) / 180.0f;
                    paint->drawLine(-960*cos((radiany)),0,960,0);



                            pen.setColor(QColor(0, 255, 0));
                            pen.setWidth(3);
                            paint->setPen(pen);
                    for(int k=0;k<180;k++)
                    {
                     iAngle=k;
                    radiany =( iAngle * M_PI ) / 180.0f;
                 //   paint->translate(960,1000);
                    paint->drawLine(0,0,-950*cos((radiany)),-950*sin((radiany))); // draws the line according to
                    }


                    pen.setColor(QColor(255, 0, 0));
                    pen.setWidth(4);
                    paint->setPen(pen);
                  //  paint->translate(960,1000);
                    for(int k=0;k<180;k++)
                    {
                        iAngle=k;
                int    pixsDistance = iDistance*22.5; // covers the distance from the sensor from cm to pixels
                    // limiting the range to 40 cms
                    if(iDistance<40){
                      // draws the object according to the angle and the distance
                        radiany =( iAngle * M_PI ) / 180.0f;
                    paint->drawLine(pixsDistance*cos((radiany)),-pixsDistance*sin((radiany)),950*cos((radiany)),-950*sin((radiany)));
                    }
                    }

       //     arc(0,0,(width-width*0.0625),(width-width*0.0625),PI,TWO_PI);
        //    arc(0,0,(width-width*0.27),(width-width*0.27),PI,TWO_PI);
       //     arc(0,0,(width-width*0.479),(width-width*0.479),PI,TWO_PI);
       //     arc(0,0,(width-width*0.687),(width-width*0.687),PI,TWO_PI);
            // draws the angle lines
       //     line(-width/2,0,width/2,0);
      //      line(0,0,(-width/2)*cos(radians(30)),(-width/2)*sin(radians(30)));
      //      line(0,0,(-width/2)*cos(radians(60)),(-width/2)*sin(radians(60)));
      //      line(0,0,(-width/2)*cos(radians(90)),(-width/2)*sin(radians(90)));
    //        line(0,0,(-width/2)*cos(radians(120)),(-width/2)*sin(radians(120)));
     //       line(0,0,(-width/2)*cos(radians(150)),(-width/2)*sin(radians(150)));
     //       line((-width/2)*cos(radians(30)),0,width/2,0);

      scene2->addPixmap(*pix); // Moved this line

}


void MainWindow::rysujRadar_siatka()
{




    pen.setColor(QColor(0, 0, 255));
    paint->setPen(pen);
    paint->setBrush(QColor(0, 0, 255));
    paint->setBackgroundMode(Qt::OpaqueMode);
    pen.setWidth(4);
    paint->setPen(pen);



  int  start_angle=0   *16;
int     end_angle= 180   *16;
paint->translate(15,98);
  // draws the arc lines
  paint->drawArc(0,0,(width-width*0.0625),(width-width*0.0625),start_angle,end_angle-start_angle);
  paint->translate(-15,-98);

  paint->translate(64,145);
  paint->drawArc(0,0,(width-width*0.27),(width-width*0.27),start_angle,end_angle-start_angle);
  paint->translate(-64,-145);

  paint->translate(112,195);
  paint->drawArc(0,0,(width-width*0.479),(width-width*0.479),start_angle,end_angle-start_angle);
  paint->translate(-112,-195);

  paint->translate(160,243);
  paint->drawArc(0,0,(width-width*0.687),(width-width*0.687),start_angle,end_angle-start_angle);
  paint->translate(-160,-243);
  // draws the angle lines





  paint->translate(width/2,height-height*0.074);

   paint->drawLine(-width/1.99,0,width/1.99,0);
   radiany =( 30 * M_PI ) / 180.0f;
   paint->drawLine(0,0,(-width/1.9)*cos(radiany),(-width/1.9)*sin(radiany));
   radiany =( 60 * M_PI ) / 180.0f;
   paint->drawLine(0,0,(-width/1.9)*cos(radiany),(-width/1.9)*sin(radiany));
   radiany =( 90 * M_PI ) / 180.0f;
   paint->drawLine(0,0,(-width/1.9)*cos(radiany),(-width/1.9)*sin(radiany));
   radiany =( 120 * M_PI ) / 180.0f;
   paint->drawLine(0,0,(-width/1.9)*cos(radiany),(-width/1.9)*sin(radiany));
   radiany =( 150 * M_PI ) / 180.0f;
   paint->drawLine(0,0,(-width/1.9)*cos(radiany),(-width/1.9)*sin(radiany));
   radiany =( 30 * M_PI ) / 180.0f;
   paint->drawLine((-width/1.9)*cos(radiany),0,width/1.9,0);
   scene2->addPixmap(*pix); // Moved this line

}

void  MainWindow::drawObject(int kat,int dystans_radar)
{
    pen.setColor(QColor(255, 0, 0));
    paint->setPen(pen);
   // paint->setBrush(QColor(255, 0, 0));
   // paint->setBackgroundMode(Qt::OpaqueMode);
    pen.setWidth(4);
    paint->setPen(pen);

   // paint->translate(width/2,height-height*0.074);


//  int pixsDistance = dystans_radar*((height-height*0.1666)*0.025); // covers the distance from the sensor from cm to pixels
  // limiting the range to 40 cms

    int pixsDistance;

    if(dystans_radar<=5 && dystans_radar>=0 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.03);
    if(dystans_radar<=10 && dystans_radar>5 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.027);
    if(dystans_radar<=15 && dystans_radar>10 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.0230);
    if(dystans_radar<=20 && dystans_radar>15 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.0215);
    if(dystans_radar<=25 && dystans_radar>20 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.0205);
    if(dystans_radar<=30 && dystans_radar>25 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.020);
    if(dystans_radar<=35 && dystans_radar>30 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.0195);
    if(dystans_radar<=40 && dystans_radar>35 )
     pixsDistance = dystans_radar*((height-height*0.1666)*0.019);

    if(dystans_radar<=40){
    // draws the object according to the angle and the distance
       radiany =( kat * M_PI ) / 180.0f;
  paint->drawLine(pixsDistance*cos(radiany),-pixsDistance*sin(radiany),(width-width*0.505)*cos(radiany),-(width-width*0.505)*sin(radiany));
  scene2->addPixmap(*pix); // Moved this line
  }

}

void  MainWindow::drawLine_radar(int kat) {
    pen.setColor(QColor(0, 255, 0));
  //  paint->setBrush(QColor(0, 255, 0));
  //  paint->setBackgroundMode(Qt::OpaqueMode);
    pen.setWidth(4);
    paint->setPen(pen);


 // paint->translate(width/2,height-height*0.074); // moves the starting coordinats to new location
  radiany =( kat * M_PI ) / 180.0f;
 // paint->drawLine(0,0,(height-height*0.12)*cos(radiany),-(radiany)*sin(radiany)); // draws the line according to the angle
  paint->drawLine(0,0,(width-width*0.505)*cos((radiany)),-(width-width*0.505)*sin((radiany)));
  scene2->addPixmap(*pix); // Moved this line
}




void MainWindow::drawText() { // draws the texts on the screen


  if(iDistance>40) {
  noObject = "Out of Range";
  }
  else {
  noObject = "In Range";
  }



  pen.setWidth(1);
  paint->setPen(pen);

 // paint->drawRect(0, height-height*0.0648, width, height);

  pen.setColor(QColor(0, 0, 255));
  paint->setPen(pen);
  paint->setBrush(QColor(0, 0, 255));

  //textSize(25);
  paint->translate(0,20);
  paint->setFont(QFont("Arial", 9, QFont::Bold));
  paint->drawText(width-width*0.3854,height-height*0.0833,"10cm");
  paint->drawText(width-width*0.281,height-height*0.0833,"20cm");
  paint->drawText(width-width*0.177,height-height*0.0833,"30cm");
  paint->drawText(width-width*0.0729,height-height*0.0833,"40cm");
  paint->translate(-0,-20);
 // textSize(40);
 // text("Object: " + noObject, width-width*0.875, height-height*0.0277);
 // text("Angle: " + iAngle +" °", width-width*0.48, height-height*0.0277);
 // text("Distance: ", width-width*0.26, height-height*0.0277);
//  if(iDistance<40) {
//  text("        " + iDistance +" cm", width-width*0.225, height-height*0.0277);
//  }
//  textSize(25);
//  fill(98,245,60);
  radiany =( 30 * M_PI ) / 180.0f;
  qDebug() << "x:" << (width-width*0.4994)+width/2*cos(radiany) << "y:" << (height-height*0.0907)-width/2*sin(radiany);


  radiany =( 30 * M_PI ) / 180.0f;
  paint->translate((width-width*0.48)+width/2*cos(radiany),(height-height*0.13)-width/2*sin(radiany));
  paint->rotate(60);
  paint->drawText(0,0,"30°");
  paint->rotate(-60);
  paint->translate(-((width-width*0.48)+width/2*cos(radiany)),-((height-height*0.13)-width/2*sin(radiany)));


  radiany =( 60 * M_PI ) / 180.0f;
  paint->translate((width-width*0.5)+width/2*cos(radiany),(height-height*0.13)-width/2*sin(radiany));
  paint->rotate(30);
  paint->drawText(0,0,"60°");
  paint->rotate(-30);
  paint->translate(-((width-width*0.5)+width/2*cos(radiany)),-((height-height*0.13)-width/2*sin(radiany)));


  radiany =( 90 * M_PI ) / 180.0f;
  paint->translate((width-width*0.51)+width/2*cos(radiany),(height-height*0.13)-width/2*sin(radiany));
  paint->rotate(0);
  paint->drawText(0,0,"90°");
  paint->rotate(-0);
  paint->translate(-((width-width*0.51)+width/2*cos(radiany)),-((height-height*0.13)-width/2*sin(radiany)));

  radiany =( 120 * M_PI ) / 180.0f;
  paint->translate((width-width*0.55)+width/2*cos(radiany),(height-height*0.10)-width/2*sin(radiany));
  paint->rotate(-30);
  paint->drawText(0,0,"120°");
  paint->rotate(30);
  paint->translate(-((width-width*0.55)+width/2*cos(radiany)),-((height-height*0.10)-width/2*sin(radiany)));

  radiany =( 150 * M_PI ) / 180.0f;
  paint->translate((width-width*0.55)+width/2*cos(radiany),(height-height*0.07)-width/2*sin(radiany));
  paint->rotate(-60);
  paint->drawText(0,0,"150°");
  paint->rotate(60);
  paint->translate(-((width-width*0.55)+width/2*cos(radiany)),-((height-height*0.07)-width/2*sin(radiany)));


    scene2->addPixmap(*pix); // Moved this line
}


void MainWindow::radar_hcsr04(int nr_radaru[],int dystans[])
{
    //rysujRadar( 30, 70, 400, 400,0,180,QColor(255, 0, 0)); // x 230 y 270  // 230 270
    //rysujRadar( 130, 170, 200, 200,0,36,QColor(0, 255, 0)); // x 230 y 270
    //rysujRadar( 130, 170, 200, 200,36,72,QColor(0, 255, 0)); // x 230 y 270
    //rysujRadar( 130, 170, 200, 200,72,108,QColor(0, 255, 0)); // x 230 y 270
    //rysujRadar( 130, 170, 200, 200,108,144,QColor(0, 255, 0)); // x 230 y 270
    //rysujRadar( 130, 170, 200, 200,144,180,QColor(0, 255, 0)); // x 230 y 270

    scene2->backgroundBrush();
    scene2->clear();


    for(int i=0;i<=180;i=i+10)
    drawLine_radar(i);
    for(int j=0;j<5;j++)
    {
    switch (nr_radaru[j]) {
    case 1:
        for(int i=0;i<=30;i=i+10)
          drawObject( i,dystans[j]) ;
        break;
    case 2:
        for(int i=30;i<=70;i=i+10)
          drawObject( i,dystans[j]) ;
        break;
    case 3:
        for(int i=70;i<=110;i=i+10)
          drawObject( i,dystans[j]) ;
        break;
    case 4:
        for(int i=110;i<=140;i=i+10)
          drawObject( i,dystans[j]) ;
        break;
    case 5:
        for(int i=140;i<=180;i=i+10)
          drawObject( i,dystans[j]) ;
        break;
    default:
        break;
    }


}


    /*
    for(int i=0;i<18;i++)
      drawObject( i,0) ;

    for(int i=18;i<36;i++)
      drawObject( i,5) ;
    for(int i=36;i<54;i++)
      drawObject( i,10) ;
    for(int i=54;i<72;i++)
      drawObject( i,15) ;
    for(int i=72;i<90;i++)
      drawObject( i,20) ;
    for(int i=90;i<108;i++)
      drawObject( i,25) ;
    for(int i=108;i<126;i++)
      drawObject( i,30) ;
    for(int i=126;i<144;i++)
      drawObject( i,35) ;
    for(int i=144;i<162;i++)
      drawObject( i,40) ;
    for(int i=162;i<180;i++)
      drawObject( i,45) ;
    */
//    for(int i=0;i<90;i++)
  //    drawObject( i,i%41) ;

 //   drawText();
 //   rysujRadar_siatka();

  //  for(int i=0;i<180;i++)
  //  drawLine_radar(i);

//    for(int i=0;i<108;i++)
  //    drawObject( i,i%32) ;
    //drawText();
    //rysujRadar_siatka();

}


void MainWindow::keyPressEvent(QKeyEvent* event){
    printf("\nkey event in board: %i", event->key());


    if (event->key() == Qt::Key_W)
        {
        on_pushButton_5_clicked();
        }
    if (event->key() == Qt::Key_S)
        {
        on_pushButton_2_clicked();
        }
    if (event->key() == Qt::Key_A)
        {
        on_pushButton_4_clicked();
        }
    if (event->key() == Qt::Key_D)
        {
        on_pushButton_3_clicked();
        }
    if (event->key() == Qt::Key_X)
        {
        on_pushButton_clicked();
        }

}


void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    if(position%15==0)
    {
   QString poz;
   poz="10,";
   poz+=QString("%1").arg(position);
    qDebug() << "1a:  " << poz ;
    uaktualnij_dane(poz);
    }
}

void MainWindow::on_pushButton_17_clicked()
{
    flaga_omijanie_przeszkod=true;
    flaga_jazda_do_celu=false;
    flaga_omijanie_przeszkod_jazda_do_celu=false;
    flaga_planowanie_jazda_do_celu_fuzzy=false;
    fuzylogic_jedz();
}

void MainWindow::on_pushButton_16_clicked()
{
    int numer[5];
    int dystans[5];
    numer[0]=1;
    dystans[0]=10 ;
   // numer[1]=2;
   // dystans[1]=10 ;
    numer[2]=3;
    dystans[2]=10 ;
    numer[3]=4;
    dystans[3]=10 ;
   // numer[4]=5;
    //dystans[4]=10 ;
  //  radar_hcsr04(numer,dystans );

   // fuzylogic_jedz_do_celu_po_prostej();

    //fuzylogic_jedz();


   // qDebug() << "1a:  " << poz_x_jada_do_celu_po_prostej ;
  //  qDebug() << "2a:  " << poz_y_jada_do_celu_po_prostej ;

   //  kierunek_jazdy();

  //  uaktualnij_dane("15,1");


//flaga_omijanie_przeszkod_jazda_do_celu=true;
//fuzylogic_cel();
//flaga_omijanie_przeszkod=true;
//fuzylogic_jedz();


  //   uaktualnij_dane("15,5*0*100*-90*100*-90*100*-90*100*-90*1");
  //   uaktualnij_dane("15,5*0*100*90*100*90*100*90*100*90*1");
     uaktualnij_dane("15,1*360*1");
     uaktualnij_dane("15,1*-360*1");
//uaktualnij_dane("15,1*0*400");


}

void MainWindow::kierunek_jazdy()
{
    // jazda do tylu -2
    // jazda w prawo -1;
    // jazda prosto 0
    // jazda w lewo +1;

    // skret w prawo -180
    // skret w prawo -90;
    // skret w lewo   90
    // jazda prosto   0

    int kierunek=0;

    int Y= poz_y_jada_do_celu_po_prostej-poz_y_jada_do_celu_po_prostej_old;
    int X= poz_x_jada_do_celu_po_prostej-poz_x_jada_do_celu_po_prostej_old;

    if(Y == 0 && X > 0)
    {
        qDebug() << "jazda prosto 0  " ;
        kierunek_jazdy_robota=0;
    }
    if(Y == 0 && X < 0)
    {
        qDebug() << "jazda do tyłu -2  " ;
        kierunek_jazdy_robota=-2;
    }
    if(Y > 0 && X == 0)
    {
        qDebug() << "jazda prawo -1  " ;
        kierunek_jazdy_robota=-1;
    }
    if(Y < 0 && X == 0)
    {
        qDebug() << "jazda lewo  +1  " ;
        kierunek_jazdy_robota=1;
    }

    int dlugosc_odcinka = sqrt(X*X + Y*Y);
    qDebug() << "dlugosc odcinka  " << dlugosc_odcinka ;
    if (kierunek_jazdy_robota==kierunek_jazdy_robota_old){
        kierunek=0;
        qDebug() << "jedz prosto  " <<kierunek  << " " << dlugosc_odcinka ;
        }
    if (kierunek_jazdy_robota!=kierunek_jazdy_robota_old)
    {
        if (kierunek_jazdy_robota_old==0 && kierunek_jazdy_robota == -2){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
        }
        if (kierunek_jazdy_robota_old==0 && kierunek_jazdy_robota == -1){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==0 && kierunek_jazdy_robota == 1){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }

        if (kierunek_jazdy_robota_old==-2 && kierunek_jazdy_robota == 0){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-2 && kierunek_jazdy_robota == -1){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-2 && kierunek_jazdy_robota == 1){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }

        if (kierunek_jazdy_robota_old==-1 && kierunek_jazdy_robota == 0){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-1 && kierunek_jazdy_robota == -2){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-1 && kierunek_jazdy_robota == 1){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }

        if (kierunek_jazdy_robota_old==1 && kierunek_jazdy_robota == 0){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==1 && kierunek_jazdy_robota == -2){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==1 && kierunek_jazdy_robota == -1){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
    }

    kierunek_jazdy_po_prostej[licznik_jazdy_po_prostej]=kierunek;
    odleglosc_jazdy_po_prostej[licznik_jazdy_po_prostej]=dlugosc_odcinka;

    qDebug() << "kierunek  " <<kierunek  << " odleglosc  " << dlugosc_odcinka ;

    poz_x_jada_do_celu_po_prostej_old=poz_x_jada_do_celu_po_prostej;
    poz_y_jada_do_celu_po_prostej_old=poz_y_jada_do_celu_po_prostej;
    kierunek_jazdy_robota_old=kierunek_jazdy_robota;
    licznik_jazdy_po_prostej++;
    // if(poz_x_jada_do_celu_po_prostej != poz_x_jada_do_celu_po_prostej_old)
  //          kierunek_jazdy_po_prostej[licznik_jazdy_po_prostej]=0;
   // if(poz_y_jada_do_celu_po_prostej != poz_y_jada_do_celu_po_prostej_old)
   //         kierunek_jazdy_po_prostej[licznik_jazdy_po_prostej]=0;

}

void MainWindow::on_Jedz_trasa_clicked()
{
    QString dane_mapy;
    dane_mapy="15,";
    dane_mapy+=QString::number(licznik_jazdy_po_prostej);
    dane_mapy+="*";
    for(int i=0;i<licznik_jazdy_po_prostej;i++)
    {
      dane_mapy+=QString::number(kierunek_jazdy_po_prostej[i]);
      dane_mapy+="*";
      dane_mapy+=QString::number(odleglosc_jazdy_po_prostej[i]);
      dane_mapy+="*";
    }
   dane_mapy=dane_mapy.left(dane_mapy.length()-1);
   qDebug() << "1aasdasd:  " << dane_mapy ;
    uaktualnij_dane(dane_mapy);
}

void MainWindow::on_pushButton_9_clicked()
{
    int xx,yy,szer,wys;
    for(int j=0; j<numer_przeszkody;j++)
    {


    xx=przeszkoda[j].x_p;
    yy=przeszkoda[j].y_p;
    szer=przeszkoda[j].x_k-xx;
    wys=przeszkoda[j].y_k-xx;
    qDebug() << "1aasdasd1:  " << xx << " " << yy << " " <<szer  << " " << wys ;

    rysujPrzeszkode( xx, yy, szer, wys,colorllinewhite);

    przeszkoda[j].x_p=0;
    przeszkoda[j].y_p=0;
    przeszkoda[j].x_k=0;
    przeszkoda[j].y_k=0;

    }
    //rysujPrzeszkode( 20, 20, 20, 20,colorllinewhite);
        numer_przeszkody=0;
        drawCross(colorlline);

}

void MainWindow::on_pushButton_18_clicked()
{
    flaga_rysowanie_przeszkod=true;

    if(ui->checkBox_3->isChecked())
    {

wczytaj_dane_z_pliku_przeszkody();
bool ok;
int xx,yy,szer,wys;
int liczba_przeszkod_mapa= tab_przeszkody_plik[0].section(' ', 0, 0).toInt(&ok);

//int porowanie=tab_fuzzy_cel[i].section(' ', 0, 0).toDouble(&ok);

for(int j=1; j<=liczba_przeszkod_mapa;j++)
{
     xx=tab_przeszkody_plik[j].section(' ', 0, 0).toInt(&ok);
     yy=tab_przeszkody_plik[j].section(' ', 1, 1).toInt(&ok);
     szer=tab_przeszkody_plik[j].section(' ', 2, 2).toInt(&ok);
     wys=tab_przeszkody_plik[j].section(' ', 3, 3).toInt(&ok);

przeszkoda[numer_przeszkody].x_p=xx;
przeszkoda[numer_przeszkody].y_p=yy;
przeszkoda[numer_przeszkody].x_k=xx+szer;
przeszkoda[numer_przeszkody].y_k=xx+wys;
numer_przeszkody++;

    rysujPrzeszkode( xx, yy, szer, wys,colorrect);
}

    drawCross(colorlline);

    }



}

void MainWindow::on_pushButton_19_clicked()
{
     flaga_omijanie_przeszkod=false;
     flaga_jazda_do_celu=true;
     flaga_omijanie_przeszkod_jazda_do_celu=false;
     flaga_planowanie_jazda_do_celu_fuzzy=false;
    fuzylogic_cel();
}

void MainWindow::on_pushButton_15_clicked()
{

    flaga_jazda_do_celu=false;
    flaga_omijanie_przeszkod_jazda_do_celu=true;
    flaga_planowanie_jazda_do_celu_fuzzy=false;


    fuzylogic_cel();
    flaga_omijanie_przeszkod=true;
}

void MainWindow::on_pushButton_20_clicked()
{
    int xx[1000],yy[1000],licznik=0;
    int z1,z2,z3,z4,licznik_z=0;
    int xxx[1000],yyy[1000],licznik_xxx=1;
    int  dlugosc_odcinka[1000], kat_nachylenia,kat_nachylenia_old[1000];
    double wsp, Y, X;
    //z=6;

    flaga_omijanie_przeszkod=false;
    flaga_jazda_do_celu=false;
    flaga_omijanie_przeszkod_jazda_do_celu=false;
    flaga_planowanie_jazda_do_celu_fuzzy=true;

/*    for(int i=0; i<1000; i=i+1)
    {
        xx[i]=0;
        yy[i]=0;
    }
*/

            //licznik_traj_robot;
    //    traj_robot[i].x_pos=x;
    //    traj_robot[i].y_pos=y;

    if(ui->checkBox_4->isChecked())
    {
// wersja dopbra wybieranie co ile z menu
for(int i=0; i<=licznik_traj_robot; i=i+ui->spinBox_9->value())
{
 xx[licznik]=traj_robot[i].x_pos;
 yy[licznik]=traj_robot[i].y_pos;
rysujPrzeszkode( xx[licznik], yy[licznik], 5, 5,color_check_point);
qDebug() << "xx  " <<xx[licznik]  << " yy  " << yy[licznik] ;
licznik++;
}
licznik--;
xx[licznik]=met.x_pos;
yy[licznik]=met.y_pos;


    for(int i=0; i<=licznik; i=i+1)
    {

       // xg=xx[i];
       // yg=-yy[i];


         xg_planowanie[i]=xx[i];
         yg_planowanie[i]=-yy[i];
         licznik_xg_yg++;

        qDebug() << "xx  " << xx[i] << " yy " << yy[i] ;
        qDebug() << "qaz  " << i << " " << z << " xg " << xg << " yg " << yg;
       // do
         //   {
                qDebug() << "zzz  " << i << " " << z << " xg " << xg << " yg " << yg;

           // }while(z>5);

    }

     xg=xg_planowanie[0];
     yg=yg_planowanie[0];
licznik_xg_yg--;
     for(int i=0; i<=licznik_xg_yg; i=i+1)
     {
         qDebug() << "xg_planowanie  " << xg_planowanie[i] << " yg_planowanie " << yg_planowanie[i] << " " << i ;
     }


    fuzylogic_cel();
    }

    if(ui->checkBox_5->isChecked() || ui->checkBox_6->isChecked())
    {

for(int i=0; i<=licznik_traj_robot; i=i+1)
{
    z1=traj_robot[i].x_pos;
    z2=traj_robot[i+10].x_pos;
    z3=traj_robot[i].y_pos;
    z4=traj_robot[i+10].y_pos;
 if(abs(z2-z1)>1 && abs(z3-z4)>1 )
 {
 xx[licznik_z]=traj_robot[i].x_pos;
 yy[licznik_z]=traj_robot[i].y_pos;
 qDebug() << "xx1  " <<xx[licznik_z]  << " yy1  " << yy[licznik_z] << " licznik_traj_robot  " << licznik_traj_robot ;;
 licznik_z++;

 }
}

for(int i=0; i<licznik_z; i=i+10)
{
    z1=xx[i]+yy[i];
    for(int j=i; j<=i+10; j=j+1)
    {
    z2=xx[j]+yy[j];

    if(abs(z2-z1)>20 )//if(abs(z2-z1)>30 )
    {
    xxx[licznik_xxx]=xx[i];
    yyy[licznik_xxx]=yy[i];
    licznik_xxx++;
    xxx[licznik_xxx]=xx[j];
    yyy[licznik_xxx]=yy[j];
    qDebug() << "xx2  " <<xxx[licznik_xxx]  << " yy2  " << yyy[licznik_xxx] << " licznik_z  " << licznik_z ;
    licznik_xxx++;



    break;
    }

    }

}

xxx[0]=start_robota.x_pos;
yyy[0]=start_robota.y_pos;
xxx[licznik_xxx-1]=met.x_pos;
yyy[licznik_xxx-1]=met.y_pos;
//licznik_xxx++;
for(int i=0; i<licznik_xxx; i=i+1)
{
rysujPrzeszkode( xxx[i], yyy[i], 5, 5,color_check_point);
qDebug() << "xx3  " <<xxx[i]  << " yy3  " << yyy[i] << " licznik_xxx  " << licznik_xxx ;
}




if(ui->checkBox_6->isChecked()==false)
{
    for(int i=0; i<licznik_xxx; i=i+1)
    {
      //  xg=xxx[i];
      //  yg=-yyy[i];

        xg_planowanie[i]=xxx[i];
        yg_planowanie[i]=-yyy[i];
        licznik_xg_yg++;
      //  qDebug() << "xx  " << xxx[i] << " yy " << yyy[i] ;
      //  qDebug() << "qaz  " << i << " " << z << " xg " << xg << " yg " << yg;
       // do
        //    {
             //   qDebug() << "zzz  " << i << " " << z << " xg " << xg << " yg " << yg;

           // }while(z>5);

    }

    xg=xg_planowanie[0];
    yg=yg_planowanie[0];
licznik_xg_yg--;
    for(int i=0; i<=licznik_xg_yg; i=i+1)
    {
        qDebug() << "xg_planowanie  " << xg_planowanie[i] << " yg_planowanie " << yg_planowanie[i] << " " << i ;
    }

    fuzylogic_cel();
}





    }



    if(ui->checkBox_6->isChecked())
    {

        QString dane_mapy;
        dane_mapy="15,";
        dane_mapy+=QString::number(licznik_xxx-1);
        dane_mapy+="*";


        for(int i=1; i<licznik_xxx; i=i+1)
        {
         Y= yyy[i]-yyy[i-1];
         X= xxx[i]-xxx[i-1];
         wsp = (Y/X);
         kat_nachylenia=qAtan(wsp)*180/pi;
         dlugosc_odcinka[i] = sqrt(X*X + Y*Y);
         kat_nachylenia_old[i]=kat_nachylenia;
         qDebug() << "Y  " << Y;
         qDebug() << "X  " << X;
        qDebug() << "współczynnik kierunkowy  " << wsp;
        qDebug() << "kat nachylenia  " << kat_nachylenia;
        qDebug() << "dlugosc odcinka  " << dlugosc_odcinka[i] ;
        }

        for(int i=1; i<licznik_xxx; i=i+1)
        {
            if(i>1 &&  kat_nachylenia_old[i-1]>= 0 &&  kat_nachylenia_old[i]>= 0 )
            {
                kat_nachylenia=kat_nachylenia_old[i-1]-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i>1 &&  kat_nachylenia_old[i-1]>= 0 &&  kat_nachylenia_old[i]< 0)
            {
                kat_nachylenia=-kat_nachylenia_old[i]-kat_nachylenia_old[i-1];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i>1 &&  kat_nachylenia_old[i-1]< 0 &&  kat_nachylenia_old[i]>= 0)
            {
                kat_nachylenia=-kat_nachylenia_old[i-1]-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i>1 &&  kat_nachylenia_old[i-1]< 0 &&  kat_nachylenia_old[i]< 0)
            {
                kat_nachylenia=kat_nachylenia_old[i-1]-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }
            if(i==1)
            {
                kat_nachylenia=-kat_nachylenia_old[i];
                dane_mapy+=QString::number(kat_nachylenia);
            }

        dane_mapy+="*";
        dane_mapy+=QString::number(dlugosc_odcinka[i]);
        dane_mapy+="*";

        }

       dane_mapy=dane_mapy.left(dane_mapy.length()-1);
       qDebug() << "1aasdasd:  " << dane_mapy ;
       uaktualnij_dane(dane_mapy);


    }


 //   flaga_omijanie_przeszkod_jazda_do_celu=false;
  //  fuzylogic_cel();
}



void MainWindow::kierunek_jazdy_fuzzy_planowanie()
{
    // jazda do tylu -2
    // jazda w prawo -1;
    // jazda prosto 0
    // jazda w lewo +1;

    // skret w prawo -180
    // skret w prawo -90;
    // skret w lewo   90
    // jazda prosto   0

    int kierunek=0;

    int Y= poz_y_jada_do_celu_po_prostej-poz_y_jada_do_celu_po_prostej_old;
    int X= poz_x_jada_do_celu_po_prostej-poz_x_jada_do_celu_po_prostej_old;

    if(Y == 0 && X > 0)
    {
        qDebug() << "jazda prosto 0  " ;
        kierunek_jazdy_robota=0;
    }
    if(Y == 0 && X < 0)
    {
        qDebug() << "jazda do tyłu -2  " ;
        kierunek_jazdy_robota=-2;
    }
    if(Y > 0 && X == 0)
    {
        qDebug() << "jazda prawo -1  " ;
        kierunek_jazdy_robota=-1;
    }
    if(Y < 0 && X == 0)
    {
        qDebug() << "jazda lewo  +1  " ;
        kierunek_jazdy_robota=1;
    }

    int dlugosc_odcinka = sqrt(X*X + Y*Y);
    qDebug() << "dlugosc odcinka  " << dlugosc_odcinka ;
    if (kierunek_jazdy_robota==kierunek_jazdy_robota_old){
        kierunek=0;
        qDebug() << "jedz prosto  " <<kierunek  << " " << dlugosc_odcinka ;
        }
    if (kierunek_jazdy_robota!=kierunek_jazdy_robota_old)
    {
        if (kierunek_jazdy_robota_old==0 && kierunek_jazdy_robota == -2){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
        }
        if (kierunek_jazdy_robota_old==0 && kierunek_jazdy_robota == -1){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==0 && kierunek_jazdy_robota == 1){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }

        if (kierunek_jazdy_robota_old==-2 && kierunek_jazdy_robota == 0){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-2 && kierunek_jazdy_robota == -1){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-2 && kierunek_jazdy_robota == 1){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }

        if (kierunek_jazdy_robota_old==-1 && kierunek_jazdy_robota == 0){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-1 && kierunek_jazdy_robota == -2){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==-1 && kierunek_jazdy_robota == 1){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }

        if (kierunek_jazdy_robota_old==1 && kierunek_jazdy_robota == 0){
            kierunek=-90;
            qDebug() << "skrec w prawo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==1 && kierunek_jazdy_robota == -2){
            kierunek=90;
            qDebug() << "skrec w lewo o 90 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
        if (kierunek_jazdy_robota_old==1 && kierunek_jazdy_robota == -1){
            kierunek=-180;
            qDebug() << "skrec w prawo o 180 stopni  " <<kierunek  << " " << dlugosc_odcinka ;
            }
    }

    kierunek_jazdy_po_prostej[licznik_jazdy_po_prostej]=kierunek;
    odleglosc_jazdy_po_prostej[licznik_jazdy_po_prostej]=dlugosc_odcinka;

    qDebug() << "kierunek  " <<kierunek  << " odleglosc  " << dlugosc_odcinka ;

    poz_x_jada_do_celu_po_prostej_old=poz_x_jada_do_celu_po_prostej;
    poz_y_jada_do_celu_po_prostej_old=poz_y_jada_do_celu_po_prostej;
    kierunek_jazdy_robota_old=kierunek_jazdy_robota;
    licznik_jazdy_po_prostej++;
    // if(poz_x_jada_do_celu_po_prostej != poz_x_jada_do_celu_po_prostej_old)
  //          kierunek_jazdy_po_prostej[licznik_jazdy_po_prostej]=0;
   // if(poz_y_jada_do_celu_po_prostej != poz_y_jada_do_celu_po_prostej_old)
   //         kierunek_jazdy_po_prostej[licznik_jazdy_po_prostej]=0;

}












//int liczniki=0;
void MainWindow::fuzylogic_linefallower()
{

    /*  czujnik lewy        1       biały  70  szary  100  czarny od 300  do 560
     *  czujnik lewy        2       biały  70  szary  50  czarny od 50  do 50
     *  czujnik środkowy    3       biały  70  szary  50  czarny od 50  do 50
     *  czujnik prawy       4       biały  70  szary  50  czarny od 50  do 50
     *  czujnik prawy       5       biały  70  szary  50  czarny od 50  do 50
     *
     *
     *
     *
     *
     * /






/*
    int fuzzy_left_linefallower=400;
    int fuzzy_front_linefallower=400;
    int fuzzy_right_linefallower=400;


    if(liczniki==0)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=400;

    }
    if(liczniki==1)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=390;

    }
    if(liczniki==2)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=390;
         fuzzy_right_linefallower=400;

    }
    if(liczniki==3)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=390;
         fuzzy_right_linefallower=390;

    }
    if(liczniki==4)
    {
         fuzzy_left_linefallower=390;
         fuzzy_front_linefallower=390;
         fuzzy_right_linefallower=390;

    }
    if(liczniki==5)
    {
         fuzzy_left_linefallower=390;
         fuzzy_front_linefallower=390;
         fuzzy_right_linefallower=400;

    }
    if(liczniki==6)
    {
         fuzzy_left_linefallower=390;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=390;

    }
    if(liczniki==7)
    {
         fuzzy_left_linefallower=390;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=400;

    }

    if(liczniki==8)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==9)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==10)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=10;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==11)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=10;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==12)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==13)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==14)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=10;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==15)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=10;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==16)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==17)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==18)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=10;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==19)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==20)
    {
         fuzzy_left_linefallower=400;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==21)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==22)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=0;

    }
    if(liczniki==23)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==24)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=400;
         fuzzy_right_linefallower=10;

    }
    if(liczniki==25)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=400;

    }
    if(liczniki==26)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=0;
         fuzzy_right_linefallower=400;

    }
    if(liczniki==27)
    {
         fuzzy_left_linefallower=0;
         fuzzy_front_linefallower=10;
         fuzzy_right_linefallower=400;

    }
    if(liczniki==28)
    {
         fuzzy_left_linefallower=10;
         fuzzy_front_linefallower=10;
         fuzzy_right_linefallower=400;

    }
    if(liczniki==29)
    {
         fuzzy_left_linefallower=(215/10.0)*10;
         fuzzy_front_linefallower=(395/10.0)*10;
         fuzzy_right_linefallower=(275/10.0)*10;

    }
*/
bool ok;



    int licznik_left=0,licznik_front=0,licznik_right=0;

    for(int i=0; i<=68921;i=i+1681)
    {

        int porowanie=tab_fuzzy_linefallower[i].section(' ', 0, 0).toDouble(&ok);
        //int porowanie=tab_left[i].toDouble(&ok);
       // qDebug() << "tab_left1 " <<tab_left[i] << " por " <<porowanie;
       // qDebug() << "tab_left2 " <<tab_left[i].toDouble(&ok) << " por " <<porowanie;
        if(porowanie<=fuzzy_left_linefallower)
        {
            licznik_left=licznik_left+1681;
            if(licznik_left>=68921)
                licznik_left=68921;

            if(fuzzy_left_linefallower>=400 && fuzzy_front_linefallower>=400 && fuzzy_right_linefallower>=400)
            {
                licznik_left=68920;
                //qDebug() << "lalalaaaaa ";
            }
        }
        else
        {
            break;
        }

    }

    for(int i=licznik_left-1; i>=0;i--)
    {

        int porowanie=tab_fuzzy_linefallower[i].section(' ', 1, 1).toDouble(&ok);
        //int porowanie=tab_front[i].toDouble(&ok);
         //qDebug() << "tab_front2 " <<tab_front[i].toDouble(&ok) << " por " <<porowanie;

        if(porowanie>fuzzy_front_linefallower)
        {
            licznik_front++;
        }
        else
        {
           // licznik_front=licznik_front+80;
            break;

        }

    }



    if(fuzzy_left_linefallower>=400 && fuzzy_front_linefallower>=400 && fuzzy_right_linefallower<=390)
    {
        //licznik_right=licznik_right;//+fuzzy_right;
   //     qDebug() << "bbbbbb ";
        licznik_front=41;
    }




    for(int i=licznik_front-1; i>=0;i--)
    {
        //qDebug() << "aaaaaaa ";


        int porowanie=tab_fuzzy_linefallower[i].section(' ', 2, 2).toDouble(&ok);
        //int porowanie=tab_right[i].toDouble(&ok);
        if(porowanie>=fuzzy_right_linefallower)
        {
           // qDebug() << "aaaaaaa1 " ;
            licznik_right++;
        }
        else
        {
            break;
        }
    }



    if(fuzzy_left_linefallower>=400 && fuzzy_front_linefallower>=400 && fuzzy_right_linefallower<=390)
    {
        //licznik_right=licznik_right;//+fuzzy_right;
    //   qDebug() << "bbbbbb7 ";
        licznik_front=0;
    }


    if(licznik_right > 40 )
        licznik_right=41-fuzzy_right_linefallower/10;
    if(fuzzy_front_linefallower == 400 && fuzzy_left_linefallower !=400 && fuzzy_right_linefallower !=400  )
        licznik_right=41-fuzzy_right_linefallower/10;
    if(fuzzy_front_linefallower == 400 && fuzzy_left_linefallower !=400 && fuzzy_right_linefallower ==400  )
        licznik_right=41-fuzzy_right_linefallower/10;
//   if(fuzzy_front_linefallower == 400 && fuzzy_left_linefallower ==400 && fuzzy_right_linefallower !=400  )
 //       licznik_right=41-fuzzy_right_linefallower;

    //int wynik=licznik_left-licznik_front-licznik_right+81;
    int wynik=licznik_left-licznik_front-licznik_right;

   lewy_silnik=tab_fuzzy_linefallower[wynik].section(' ', 3, 3);
   prawy_silnik=tab_fuzzy_linefallower[wynik].section(' ', 4, 4);
//lewy_silnik=tab_left_engine[wynik];
//prawy_silnik=tab_right_engine[wynik];
    qDebug() << "fuzzy_left " <<fuzzy_left_linefallower << "fuzzy_front " <<fuzzy_front_linefallower << "fuzzy_right "<< fuzzy_right_linefallower;
    qDebug() << "licznik left " <<licznik_left ;
   qDebug() << "licznik front " <<licznik_front ;
    qDebug() << "licznik right " <<licznik_right ;
    qDebug() << "wynik " <<wynik ;
 //   qDebug() << "lewy silnik " <<lewy_silnik << " prawy silnik " << prawy_silnik;
   // uaktualnij_dane(""+lewy_silnik+","+prawy_silnik);

    uaktualnij_dane("19,"+lewy_silnik+"*"+prawy_silnik);
    //liczniki++;
//    if (licznik_omijanie_przeszkod==10)
//    uaktualnij_dane("13,1");
// licznik_omijanie_przeszkod++;
}


void MainWindow::on_pushButton_23_clicked()
{
    flaga_linefallower=true;
    fuzylogic_linefallower();
}
