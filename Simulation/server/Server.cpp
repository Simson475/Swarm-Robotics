#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <ctype.h>
#include "nlohmann/json.hpp"
#include <sys/stat.h>
#include <bits/stdc++.h>
#include "stratego.hpp"

void readFile(int socket, std::string robotName, std::string fileName);
void perform(int socket, char* robotName, stratego::queryType type);
void createFolder(char* robotName);
void error(const char *msg) {
    perror(msg);
    exit(1);
}

int main(int argc, char *argv[]) {
    int sockfd, newsockfd, portno, pid;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;
    if (argc < 2) {
        fprintf(stderr, "ERROR, no port provided\n");
        exit(1);
    }
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    bzero((char *)&serv_addr, sizeof(serv_addr));
    portno = atoi(argv[1]);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR on binding");
    listen(sockfd, 5);
    clilen = sizeof(cli_addr);
    while (1) {
        newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);
        if (newsockfd < 0)
            error("ERROR on accept");
        pid = fork();

        if (pid == 0) {
            int objective = -1;
            read(newsockfd, &objective, sizeof(int));
            int nameSize = -1;
            read(newsockfd, &nameSize, sizeof(int));
            char name[nameSize];
            int k = read(newsockfd, &name, nameSize);
            name[k] = '\0'; // clip the entry
            if(objective) perform(newsockfd, name, stratego::queryType::stations);
            else perform(newsockfd, name, stratego::queryType::waypoints);

            std::cout << "Robot "<< name << " has received plan, socket closes"<<std::endl;
            exit(0);
        } else
            close(newsockfd);
    } /* end of while */
    close(newsockfd);
    close(sockfd);
    return 0;
}
void perform(int socket, char* robotName, stratego::queryType type) {
    std::cout << "Folder name is: " << std::string(robotName) << std::endl;
    createFolder(robotName);
    if(type == stratego::queryType::stations){
        std::cout << "Robot "<< std::string(robotName) << " requests for Stations plan"<<std::endl;
        readFile(socket, std::string(robotName), "/stations/static_config.json");
        readFile(socket, std::string(robotName), "/stations/dynamic_config.json");
    }
    else{
        std::cout << "Robot "<< std::string(robotName) << " requests for Waypoints plan"<<std::endl;
        readFile(socket, std::string(robotName), "/waypoints/static_config.json");
        readFile(socket, std::string(robotName), "/waypoints/dynamic_config.json");
    }
    std::string result = stratego::getSingleTrace(robotName, type);
    int size = result.length();
    int n ;
    n= write(socket,&size ,sizeof(int));
    if (n < 0) error("ERROR writing to socketsrv");
    n = write(socket,result.c_str() ,size);
    if (n < 0) error("ERROR writing to socketsrv");
}
void createFolder(char* robotName){
    mkdir(robotName, 0777);
    mkdir((std::string(robotName) +"/waypoints").c_str(), 0777);
    mkdir((std::string(robotName) +"/stations").c_str(), 0777);
}
void readFile(int socket, std::string robotName, std::string fileName){
    std::string path = (robotName + "/"+fileName).c_str();
    int symbols;
    FILE *fp;
    int ch = 0;
    fp = fopen(path.c_str(), "w");
    read(socket, &symbols, sizeof(int));
    char buffer[symbols];
    while (ch <= symbols-1) {
        size_t readSize = read(socket, buffer, symbols);
        buffer[readSize] = '\0';
        if (buffer[readSize-1] == '\n')
            buffer[readSize-1] = '\0';
        fprintf(fp, " %s", buffer);
        ch +=readSize;
    }
    fflush(fp);
}