#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "../lib/WGS84toCartesian.hpp"
#include "vibes.h"

using namespace std;

double lxm = -3.01468;
double lym = 48.1989;

vector<string> split(const string& str, const string& delim)
{
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}

int card(char& letter){
    if (letter == 'N' or letter == 'E'){
        return 1;
    } else if (letter == 'S' or letter == 'W'){
        return -1;
    }else{
        throw string("ERREUR : Direction Cardinale invalide");
    }
}

double to_deg(string mes){
    vector<string> tokens = split(mes, ".");
    size_t pos = tokens[0].size() -2;
    double d, m;
    d = stod(tokens[0].substr(0, pos), nullptr);
    m = stod(tokens[0].substr(pos, 2), nullptr) + stod("0." + tokens[1], nullptr);
    cout << d << " " << m << endl;
    return d + m/60.;
}

int main(int argc, char**argv){
    ifstream f("../data/data.txt");
    if(!f.is_open())
        cout << "Erreur d'ouverture de " << "position.txt" << endl;
    else{
        vibes::beginDrawing();
        vibes::newFigure("DDBoat");
        string lign, lat, lon, time;
        char card_lat, card_lon;
        double latitude, longitude;

        while (getline(f, lign)){
            istringstream input(lign);
            input >> lat >> card_lat >> lon >> card_lon >> time;
            latitude = card(card_lat) * to_deg(lat);
            longitude = card(card_lon) * to_deg(lon);
            cout << latitude << " " << longitude << endl;
            array<double, 2> pos =wgs84::toCartesian({lym, lxm}, {latitude, longitude});
            vibes::drawPoint(pos[0], pos[1], 5);
            }
        vibes::endDrawing();
        }
    return 0;
}