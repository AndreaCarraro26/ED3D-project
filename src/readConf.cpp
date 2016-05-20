#include "laser_scanner.h"


void read_config(Configuration& confData) {

	std::string confFile = "../data/Configuration.xml";
	cv::FileStorage fs;
	fs.open(confFile, cv::FileStorage::READ);

	fs["model"] >> confData.modelName;

	fs["cameraHeight"] >> confData.cameraHeight;

	fs["alphaLaser"] >> confData.alphaLaser;
	fs["laserDistance"] >> confData.laserDistance;
	fs["fanLaser"] >> confData.fanLaser;
	fs["numLaser"] >> confData.numLaser;
	confData.laserLength = confData.cameraHeight/cos(3.1416/180*confData.alphaLaser);
	
	fs["useBounds"] >> confData.useBounds; // Se true usa il range letto, altrimenti calcola quello ottimale
	fs["minY"] >> confData.minY;
	fs["maxY"] >> confData.maxY;

	fs["scanSpeed"] >> confData.scanSpeed;
	fs["fpsCam"] >> confData.fpsCam;


	fs["sensor_f_x"] >> confData.f_x;
	fs["sensor_f_y"] >> confData.f_y;
	fs["sensor_x_0"] >> confData.x_0;
	fs["sensor_y_0"] >> confData.y_0;

	fs["sensor_width"] >> confData.sensor_width;
	fs["sensor_height"] >> confData.sensor_height;

	fs["roi_height"] >> confData.roi_height;
	fs["roi_y_1"] >> confData.roi_y_1; 
	fs["roi_y_2"] >> confData.roi_y_2; 
	
	fs.release();

	cv::Mat intrinsic(3, 3, CV_64FC1);
	
	// Popolamento della matrice degli intrinsici
	intrinsic.at<double>(0, 0) = confData.f_x;
	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(0, 2) = confData.x_0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(1, 1) = confData.f_y;
	intrinsic.at<double>(1, 2) = confData.y_0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	confData.intrinsicMat = intrinsic;

}

void edit_conf(Configuration& confData){

	#ifdef _WIN32
		system("cls");
	#elif linux 
		system("clear");
	#endif

	std::cout << "MENU' DI CONFIGURAZIONE\n";
	std::cout << "-----------------------\n";
	int answer = -1;

	std::cout << "Selezionare il parametro da modificare:\n";
	std::cout << "[1] Altezza sistema telecamera/laser\n";
	std::cout << "[2] Larghezza dei laser rispetto alla telecamera\n";
	std::cout << "[3] Inclinazione dei piani laser rispetto al piano XY\n";
	std::cout << "[4] Posizione Y della telecamera iniziale\n";
	std::cout << "[5] Posizione Y della telecamera finale\n\n"<<std::flush;
	std::cin >> answer;

	float input;
	bool input_valid = false;
	while(true) {
		switch(answer) {

			case 1:
				while(!input_valid) {
					input_valid = true;
					std::cout << "Valore corrente: " << confData.cameraHeight << std::endl;
					std::cout << "Nuova altezza: "<<std::flush; std::cin >> input;
					if(std::cin.fail() || input<=0) {
						std::cin.clear();
						std::cin.ignore();
						std::cout << "Input non valido! Inserire un numero maggiore di 0\n";
						input_valid = false;
					}
				}
				confData.cameraHeight = input;
				break;

			case 2: 
				while(!input_valid) {
					input_valid = true;
					std::cout << "Valore corrente: " << confData.laserDistance << std::endl;
					std::cout << "Nuova distanza (valori accettati: da 500 a 800mm): "; std::cin >> input;
					if(std::cin.fail() || input <= 500 || input > 800) {
						std::cin.clear();
						std::cin.ignore();
						std::cout << "Input non valido! Inserire un numero compreso tra 500 e 800\n";
						input_valid = false;
					}
				}
				confData.laserDistance = input;
				break;

			case 3: 
				while(!input_valid) {
					input_valid = true;
					std::cout << "Valore corrente: " << confData.alphaLaser << std::endl;
					std::cout << "Nuovo angolo (valori accettati: da 60 a 70 gradi): "; std::cin >> input;
					if(std::cin.fail() || input <= 60 || input > 70) {
						std::cin.clear();
						std::cin.ignore();
						std::cout << "Input non valido! Inserire un numero compreso tra 60 e 70\n";
						input_valid = false;
					}

				}
				confData.alphaLaser = (int) input;
				break;
			case 4: 
				while(!input_valid) {
					input_valid = true;
					std::cout << "Valore corrente: " << confData.minY << std::endl;
					std::cout << "Nuova posizione iniziale: "; std::cin >> input; 
					if(std::cin.fail()) {
						std::cin.clear();
						std::cin.ignore();
						std::cout << "Input non valido! Inserire un numero!\n";
						input_valid = false;
					}
				}
				confData.minY = input;
				break;
			case 5: 
				while(!input_valid) {
					input_valid = true;
					std::cout << "Valore corrente: " << confData.maxY << std::endl;
					std::cout << "Nuova posizione finale: "; std::cin >> input; 
					if(std::cin.fail()) {
						std::cin.clear();
						std::cin.ignore();
						std::cout << "Input non valido! Inserire un numero!\n";
						input_valid = false;
					}
				}
				confData.maxY = input;
				break;
			default:

				std::cin.clear();
				std::cin.ignore();

				#ifdef _WIN32
					system("cls");
				#elif linux 
					system("clear");
				#endif

				std::cout << "Scelta non valida! Selezionare un numero del menu'\n\n";
				std::cout << "Selezionare il parametro da modificare:\n";
				std::cout << "[1] Altezza sistema telecamera/laser\n";
				std::cout << "[2] Larghezza dei laser rispetto alla telecamera\n";
				std::cout << "[3] Inclinazione dei piani laser rispetto al piano XY\n";
				std::cout << "[4] Posizione Y della telecamera iniziale\n";
				std::cout << "[5] Posizione Y della telecamera finale\n\n"<<std::flush;
				std::cin >> answer;
				continue;
		}
		break;
	}


}