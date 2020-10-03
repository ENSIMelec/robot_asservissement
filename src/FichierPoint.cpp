#include "FichierPoint.h"

using namespace std;
namespace pt = boost::property_tree;

vector<Point> FichierPoint::readPoints(string filename) {
    vector<Point> pts;

    string type;
    float X, Y, angle;
    float distance_tolerance;
    float angle_tolerance;
    int speed;
    bool slipping;
    //string sens, blocage;
    //double coefCourbe;
    //bool lissage, derapage, attAction;
    int timeout;
    string action;
    Controller::Trajectory mTrajectory;

    cout << "Lecture du fichier point : '" << filename << "'" << endl;
    
    pt::ptree liste;
    pt::read_json(filename, liste);

    BOOST_FOREACH(const pt::ptree::value_type &point, liste.get_child("Points")) {
        X = 0;
        Y = 0;
        angle = 0;
        distance_tolerance = 0.0;
        angle_tolerance = 0.0;
        speed = 0;
        timeout = 0;
        action = "null";
        slipping = false;

        type = point.second.get<string>("type");
		
		if(type.compare("XY_ABSOLU") == 0) {
            X = point.second.get<float>("X");
            Y = point.second.get<float>("Y");
            //angle = point.second.get<float>("THETA");
            distance_tolerance = point.second.get<float>("distance_tolerance");
            angle_tolerance = point.second.get<float>("angle_tolerance");
            speed = point.second.get<int>("speed");
            timeout = point.second.get<int>("timeout");
            action = point.second.get<string>("action");
            slipping = point.second.get<string>("slipping"); // dérapage ?
            mTrajectory = Controller::Trajectory::XY_ABSOLU;

		}
        if(type.compare("CALIB_X") == 0) {
            X = point.second.get<float>("X");
            angle = point.second.get<float>("THETA");
            timeout = point.second.get<int>("timeout");
            action = point.second.get<string>("action");
            mTrajectory = Controller::Trajectory::CALIB_X;
        }
        if(type.compare("CALIB_Y") == 0) {
            Y = point.second.get<float>("Y");
            angle = point.second.get<float>("THETA");
            timeout = point.second.get<int>("timeout");
            action = point.second.get<string>("action");
            mTrajectory = Controller::Trajectory::CALIB_Y;
        }
        if(type.compare("CALIB_XY") == 0) {
            X = point.second.get<float>("X");
            Y = point.second.get<float>("Y");
            angle = point.second.get<float>("THETA");
            timeout = point.second.get<int>("timeout");
            action = point.second.get<string>("action");
            mTrajectory = Controller::Trajectory::CALIB_XY;
        }
		else if (type.compare("THETA") == 0) {
            angle = point.second.get<float>("THETA");
            angle_tolerance = point.second.get<float>("angle_tolerance");
            speed = point.second.get<int>("speed");
            timeout = point.second.get<int>("timeout");
            action = point.second.get<string>("action");
            mTrajectory = Controller::Trajectory::THETA;
		}
		else if(type.compare("POSITION") == 0) {
            X = point.second.get<float>("X");
            Y = point.second.get<float>("Y");
            angle = point.second.get<float>("THETA");
            mTrajectory = Controller::Trajectory::NOTHING;
		}
        else {
            continue;//aucun type connu, on passe à l'itération suivante
        }



        // Controller
        Point p(X, Y, angle, mTrajectory);
        p.setMAction(ref(action));
        p.setSlipping(slipping);

        //Point p(type, X, Y, angle, deltaDeplacement, deltaAngle, vitesse, sens, blocage, coefCourbe, lissage, derapage, timeOut, action, attAction);
        pts.push_back(p);
    }

    // point pour locker
    Point pblocked(0,0,0,Controller::Trajectory::LOCKED);
    pts.push_back(pblocked);

    return pts;
}