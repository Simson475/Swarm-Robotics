#include "connector.hpp"

#define SERVER_ADDRESS "localhost"
#define PORT 20009

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

/**
 * 
 * 
 * @param station string
 * @param robotName string
 * @param dynamic string
 * @param folderPath string
 * 
 * @return string
*/
std::string connectStratego(std::string station, std::string robotName, std::string dynamic, std::string folderpath)
{
    int stations = (station == "Stations") ? 1 : 0;
    int sockfd, portno;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    portno = PORT;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        error("ERROR opening socket");
    server = gethostbyname(SERVER_ADDRESS);
    if (server == NULL) {
        error("ERROR, no such host\n");
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));// Set all struct bytes to 0.
    serv_addr.sin_family = AF_INET;
    // Copy the host address to the socket address
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno); // htons converts host format port address to network format (for TCP/IP networks)
    if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        error("ERROR connecting");
    } else {
        std::ofstream out2("text.txt",std::ofstream::app);
        out2<<robotName.c_str()<<" has connected to the server"<<std::endl;
    }

    int nameSize = robotName.size();
    write(sockfd, &stations, sizeof(stations)); // Inform server if it's waypoints or stations
    write(sockfd, &nameSize, sizeof(nameSize)); // Inform server the size of the robotname
    write(sockfd, robotName.c_str(), robotName.size());// Robot name
    sendFile1(sockfd, folderpath+robotName+"/static_config.json");
    sendFile(sockfd, folderpath+robotName+"/dynamic_config.json", dynamic);



    int size, n;
    read(sockfd, &size, sizeof(int));
    char buff[size+1];
    n = read(sockfd, &buff, size);
    if (n < 0) error("ERROR reading from socket");
    buff[n] = '\0';

    std::string path = path = folderpath + robotName + "/" + robotName + ((stations) ? "Stations.json" : "Waypoints.json");
    std::ofstream out(path), out2("text.txt", std::ofstream::app);
    out << std::setw(4) << buff; // Streams the buffer in chunks of 4 bytes
    out2 << robotName << " received the data from server, socket closed" << std::endl;

    close(sockfd);
    return buff;
}

/**
 * Writes the file to the socket.
 * 
 * @param sockfd int
 * @param fileName string
*/
void sendFile1(int sockfd, std::string fileName){
    int chars;

    // Open file and read to buffer (buff)
    std::ifstream file(fileName);
    if (file < 0) error("Could not open file.\n");
    std::stringstream buff;
    buff << file.rdbuf();

    // Convert buffer stringstream to a string
    std::string content( buff.str() );

    // Send the content
    chars = content.length();
    write(sockfd, &chars, sizeof(int));
    write(sockfd, content.c_str(), chars);
}

/**
 * Does not actually send the file, but just sends the chars that are not spaces in the dynamic string
 * 
 * @param sockfd int
 * @param fileName string - Not used
 * @param dynamic string
*/
void sendFile(int sockfd, std::string fileName, std::string dynamic){
    // Move all spaces to the end of the string and erase them.
    dynamic.erase(std::remove_if(dynamic.begin(), dynamic.end(), isspace), dynamic.end());
    // Send the dynamic string
    int chars = dynamic.length();
    write(sockfd, &chars, sizeof(int));
    write(sockfd, dynamic.c_str(), dynamic.length());
}
