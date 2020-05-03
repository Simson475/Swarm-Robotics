#include "connector.h"

#define SERVER_ADDRESS "localhost"
#define PORT 20009

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

std::string connectStratego(std::string station, std::string robotName, std::string dynamic)
{
    int stations;
    if(station == "Stations") stations = 1;
    else stations = 0;
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    portno = PORT;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(SERVER_ADDRESS);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    else{std::ofstream out2("text.txt",std::ofstream::app);out2<<robotName.c_str()<<" has connected to the server"<<std::endl;}

    int nameSize = robotName.size();
    write(sockfd, &stations, sizeof(int)); // inform server if it's waypoints or stations
    write(sockfd, &nameSize, sizeof(int)); // inform server the size of the robotname
    write(sockfd, robotName.c_str(), robotName.size());//robot name
        sendFile1(sockfd, "experiment/scene2/"+robotName+"/static_config.json");
    sendFile(sockfd, "experiment/scene2/"+robotName+"/dynamic_config.json",dynamic);

    
    
      int size;
  read(sockfd, &size, sizeof(int));
      char buff[size]; 

      n = read(sockfd, &buff, size);
      buff[n] = '\0';
    std::string path;
      if (stations) {
    
    path = "experiment/scene2/" + robotName + "/" + robotName + "Stations.json";
  }
  else {
    path ="experiment/scene2/" + robotName + "/" + robotName + "Waypoints.json";
  }
      std::ofstream out(path);
    out << std::setw(4) << buff;
    if (n < 0) 
         error("ERROR reading from socket");
	std::ofstream out2("text.txt",std::ofstream::app);out2<<robotName<<" received the data from server, socket closed"<<std::endl;
    close(sockfd);
    return buff;
}


void sendFile1(int sockfd, std::string fileName){
    char buffer[512];
    FILE *f;
    int words = 1;
    char c;

std::ifstream     file(fileName);
std::stringstream buff;
buff << file.rdbuf();

std::string content( buff.str() );
  words = content.length();
    //fin.close();


	write(sockfd, &words, sizeof(int));
  write(sockfd,content.c_str(),words);
    /*rewind(f);

    char ch =1 ;
       while(ch != EOF)
      {
		fscanf(f, "%s" , buffer);
		write(sockfd,buffer,512);
		ch = fgetc(f);
      }
      delete f;*/

}
void sendFile(int sockfd, std::string fileName, std::string dynamic){
    dynamic.erase(std::remove_if(dynamic.begin(), dynamic.end(), isspace), dynamic.end());
    int words = 1;
    for(auto& c: dynamic){
      //if(isspace(c)||c=='\t')
		    words++;	
    }
    words = dynamic.length();
	  write(sockfd, &words, sizeof(int));
    write(sockfd,dynamic.c_str(),dynamic.length());
}
