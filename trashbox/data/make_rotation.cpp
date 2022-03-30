#include <iostream>
#include <fstream>

using namespace std;

int main(){
    char outputfilename[256];

    uint batch =1;
    sprintf(outputfilename, "debug_data/batch_%u.dat", batch);
	std::ofstream outputfile (outputfilename, std::ios::out | std::ios::trunc);      //Output File (ofstream) 
	std::cout << "Writing details to file  : " << outputfilename << std::endl;				
    int steps = 100;
    outputfile << 1 <<" "<< 0 <<" "<< 0<< endl;
    for (int i = 1; i < steps; i++)
    {
        outputfile << 0 << " " << i << " " << i << " " << i << endl; //誤差あり観測
    }
    outputfile << 2 <<" "<< 0 <<" "<< 0<< endl;
    return 0;
}