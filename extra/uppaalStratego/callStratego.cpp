#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <json.hpp>
#include <algorithm>
#include <iostream>
#include <regex>
#include <sstream>
using namespace std;
using Json = nlohmann::json;

struct TimeValuePair {
    double time;
    int value;
};

struct SimulationTrace {
    int number;
    std::vector<TimeValuePair> values;
};
struct SimulationExpression {
    std::string name;
    vector<SimulationTrace> runs;
};


void createModel(string within, string angle, string prox, string limit, string strategoMasterModel,string strategoFinalModel, string query);
string GetStdoutFromCommand(string cmd);
int getStrategy(string out);
string createModel2(string s0, string s1, string s2, string s3,string s4);
SimulationExpression parseValue(std::istream &ss,std::string &line);
vector<SimulationExpression> parseStr(string result, int formula_number);

int main(int argc, char *argv[]){ 
    int action =2;
    string libPath = "LD_LIBRARY_PATH=/home/widok/Desktop/newestDat7/P7-kode/build/lib/";
    string uppaalPath = "~/Desktop/uppaalStratego/bin-Linux/verifyta.bin";
    string modelPath = "~/Desktop/newestDat7/P7-kode/scheduling/station_scheduling.xml";
    string queryPath ="~/Desktop/newestDat7/P7-kode/scheduling/station_scheduling.q";
    string modelPath2 = "~/Desktop/newestDat7/P7-kode/scheduling/waypoint_scheduling.xml";
    //string queryPath2 ="~/Desktop/newestDat7/P7-kode/scheduling/waypoint_scheduling.q";
    string queryPath2 ="waypoint_scheduling.q";
    string result;
    if (string(argv[2]) == "Stations"){
    result = createModel2(libPath, uppaalPath, modelPath, queryPath, string(argv[1]));
    }else{ result = createModel2(libPath, uppaalPath, modelPath2, queryPath2, string(argv[1]));
    }
    cout<< result;
    vector<SimulationExpression> parsed = parseStr(result, 2);

    vector<Json> mainObj;
    for (int i = 0; i <parsed.size(); i++){
        if (!(parsed[i].name[0] >= 65 && parsed[i].name[0] <= 90) 
        || (parsed[i].name[0] >= 97 && parsed[i].name[0] <= 122)){
            break;
        }
        Json jsonObj;
        jsonObj["name"] = parsed[i].name;
        vector<Json> subObjs;
        for (int j = 0; j <parsed[i].runs.size(); j++){
            Json subObj;
            subObj["number"] = parsed[i].runs[j].number;
            vector<Json> subSubObjs;
            for (int k= 0; k <parsed[i].runs[j].values.size(); k++){
                Json subSubObj;
                subSubObj["time"] = parsed[i].runs[j].values[k].time;
                subSubObj["value"] = parsed[i].runs[j].values[k].value;
                subSubObjs.push_back(subSubObj);
            }
            subObj["values"] = subSubObjs;
            subObjs.push_back(subObj);
        }
        jsonObj["run"] = subObjs;
        mainObj.push_back(jsonObj);
    }
    string fileName;
    if (string(argv[2]) == "Stations"){fileName = string("experiment/scene2/")+argv[1] +"/"+argv[1] + string("Stations") + string(".json");}
    else fileName = string("experiment/scene2/")+argv[1] +"/"+argv[1] + string("Waypoints") + string(".json");
    std::ofstream out(fileName);
    out << std::setw(4) << mainObj;
    return 0;
}
string createModel2(string s0, string s1, string s2, string s3, string s4){
    string complete = "cd experiment/scene2/" +s4+" && "+ s0 + " " + s1 + " " + s2 + " " + s3;
    string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    stream = popen(complete.c_str(), "r");
    if (stream) {
        while (!feof(stream))
            if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
                pclose(stream);
    }
    return data;

}


vector<SimulationExpression> parseStr(string result, int formula_number){
    const size_t startIndex = result.find("Verifying formula " + std::to_string(formula_number));
    const size_t stopIndex =
        result.find("Verifying formula " +
                    std::to_string(formula_number + 1)); // Equal to std::string::npos if not found.

    std::string formula;
    if (stopIndex == std::string::npos) {
        formula = result.substr(startIndex);
    }
    else {
        formula = result.substr(startIndex, stopIndex - startIndex);
    }

    std::stringstream ss{formula};
    vector<SimulationExpression> values;

    std::string line;
    std::getline(ss, line); // Verifying formula \d+ at <file:line>
    std::getline(ss, line); // -- Formula is (not)? satisfied.

    if (ss.eof()) {
        return values;
    }

    std::getline(ss, line);
    while (line.size() > 0) {
        values.push_back(parseValue(ss, line));
        if (ss.eof()) {
            break;
        }
    }

    return values;
}
class SimulationParseException : public std::exception {
    std::string message;

  public:
    SimulationParseException(const std::string &inmessage) : message(inmessage) {}

    const char *what() const noexcept override { return message.c_str(); }
};
SimulationExpression parseValue(std::istream &ss,std::string &line)
{
        std::string name = line.substr(0, line.length() - 1);
    std::vector<SimulationTrace> runs;

    // Read run
    std::getline(ss, line);

    std::string time;
    std::string value;

    // Matches full row of simulation results on the form: [run_number]: (time,value)
    // (time,value)... group 1 == run_number, group 2 == all time-value pairs.
    static const std::regex line_pattern{R"(\[(\d+)\]:((?: \(-?\d+(?:\.\d+)?,-?\d+\))*))"};
    // matches single (time,value) pair.
    // group 1 == time, group 2 == value.
    static const std::regex pair_pattern{R"( \((-?\d+(?:\.\d+)?),(-?\d+)\))"};

    std::smatch line_match;
    std::smatch pair_match;
    while (std::regex_match(line, line_match, line_pattern)) {
        std::vector<TimeValuePair> values;
        int run_number = std::stoi(line_match[1]);
        std::string pairs = line_match[2];

        // parse the list of value pairs
        while (std::regex_search(pairs, pair_match, pair_pattern)) {
            double time = std::stod(pair_match[1]);
            int value = std::stoi(pair_match[2]);
            values.push_back(TimeValuePair{time, value});
            pairs = pair_match.suffix();
        }

        runs.push_back(SimulationTrace{run_number, values});
        if (ss.eof()) {
            break;
        }

        std::getline(ss, line);
    }

    return SimulationExpression{name, runs};
}
/*{
    std::string name = line.substr(0, line.length() - 1);
    
    vector<SimulationTrace> runs;

    // Read run
    std::getline(ss, line);
    cout <<"linija "<<line<<endl;
    while (line.size() > 0 && line.at(0) == '[') {
        // Find all (t, n) pairs
        int state = 0;
        std::stringstream time;
        std::stringstream value;
        std::stringstream run_number;
        std::vector<TimeValuePair> values;

        // Regex for this (ignore whitespace) \[\d+\]: ( \( \d+(\.\d+)? , \d+ \) )*
        for (char &c : line) {
            if (isspace(c)) {
                continue;
            }

            switch (state) {
            case 0:
                if (c != '[') {
                    throw SimulationParseException("State 0 missing [");
                }
                state = 1;
                break;
            case 1:
                if (!isdigit(c)) {
                    throw SimulationParseException("State 1 missing digit");
                }
                run_number << c;
                state = 2;
                break;
            case 2:
                if (isdigit(c)) {
                    run_number << c;
                }
                else if (c == ']') {
                    state = 3;
                }
                else {
                    throw SimulationParseException("State 2 missing digit or ]");
                }
                break;
            case 3:
                if (c != ':') {
                    throw SimulationParseException("State 3 missing :");
                }
                state = 4;
                break;
            case 4:
                if (c != '(') {
                    throw SimulationParseException("State 4 missing (");
                }
                state = 5;
                break;
            case 5:
                if (!isdigit(c)) {
                    throw SimulationParseException("State 5 missing digit");
                }
                time << c;
                state = 6;
                break;
            case 6:
                if (isdigit(c)) {
                    time << c;
                }
                else if (c == '.') {
                    time << c;
                    state = 7;
                }
                else if (c == ',') {
                    state = 9;
                }
                else {
                    throw SimulationParseException("State 6 missing digit, . or ,");
                }
                break;
            case 7:
                if (!isdigit(c)) {
                    throw SimulationParseException("State 7 missing digit");
                }
                time << c;
                state = 8;
                break;
            case 8:
                if (isdigit(c)) {
                    time << c;
                }
                else if (c == ',') {
                    state = 9;
                }
                else {
                    throw SimulationParseException("State 8 missing digit or ,");
                }
                break;
            case 9:
                if (!isdigit(c)) {
                    throw SimulationParseException("State 9 missing digit");
                }
                value << c;
                state = 10;
                break;
            case 10:
                if (isdigit(c)) {
                    value << c;
                }
                else if (c == ')') {
                    double time_double = std::stod(time.str());
                    int value_int = std::stoi(value.str());
                    // Clear string streams
                    time.str(std::string());
                    value.str(std::string());

                    values.push_back(TimeValuePair{time_double, value_int});

                    state = 4;
                }
                else {
                    throw SimulationParseException("State 10 missing digit or )");
                }
                break;
            }
        }

        runs.push_back(SimulationTrace{std::stoi(run_number.str()), values});

        if (ss.eof()) {
            break;
        }

        // Read next run
        std::getline(ss, line);
    }

    return SimulationExpression{name, runs};
}*/
















void createModel(string within, string angle, string prox, string limit, string strategoMasterModel, string strategoFinalModel, string query){
    
    
    

    ifstream inFile(strategoMasterModel);
    ofstream outFile(strategoFinalModel);

    

    string line;
    string toReplace0 = "//HOLDER_within";
    string toReplace1 = "//HOLDER_angle";
    string toReplace2 = "//HOLDER_prox";
    string toReplace3 = "//HOLDER_limit";

    //size_t len = toReplace.length();
    bool key0= true;
    bool key1=true;
    bool key2=true;
    bool key3=true;

    
    

    while (getline(inFile, line)){
        //cout<<line<<endl;

        

        outFile<<line<<endl;
        if(key0==true || key1==true || key2==true || key3==true){
            if (key0== true && line.compare(toReplace0) == 0){ 
                outFile<<"bool within = "<<within<<";"<<endl;
                key0=false;
            } 
            else if(key1== true && line.compare(toReplace1) == 0){
                double dangle=atof(angle.c_str())+ 0.01;
                ostringstream buffdangle;
                buffdangle<<dangle;
                outFile<<"double angle = "<<buffdangle.str()<<";"<<endl;
                key1=false;
            }
            else if(key2== true && line.compare(toReplace2) == 0){
                double dprox=atof(prox.c_str())+ 0.0001;
                ostringstream buffdprox;
                buffdprox<<dprox;
                outFile<<"double prox = "<<buffdprox.str()<<";"<<endl;
                key2=false;
            }
            else if(key3== true && line.compare(toReplace3) == 0){
                double dlimit=atof(limit.c_str())+ 0.0001;
                ostringstream buffdlimit;
                buffdlimit<<dlimit;
                outFile<<"double limit = "<<buffdlimit.str()<<";"<<endl;
                key3=false;
            }
        }   
    }
    outFile.close();
    inFile.close();
}

string GetStdoutFromCommand(string cmd) {
    string data;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) {
    while (!feof(stream))
    if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
    }
    return data;
}

int getStrategy(string out){
    string str ="(0,";
    size_t found = out.find(str);
    string target = out.substr(found+3,1);
    
    

    //cout << "target: " <<  target << endl;

    //return stoi(target);
    return 0;
}
