#ifndef CONNECTOR_H
#define CONNECTOR_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <ctype.h>
#include <bits/stdc++.h>

void error(const char *msg);
std::string connectStratego(std::string station, std::string robotName, std::string dynamic);
void sendFile(int sockfd, std::string fileName, std::string dynamic);
void sendFile1(int sockfd, std::string fileName);
#endif