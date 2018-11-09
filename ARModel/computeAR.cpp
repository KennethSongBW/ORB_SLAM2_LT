#include <iostream>
#include <fstream>
#include <vector>
#include "ar.cpp"

using namespace std;

int main()
{
    ifstream fin;
    fin.open("test.txt");
    if (!fin)
    {
        cout << "Can not open parameter file\n";
        return 0;
    }
    long value;
    long number;
    vector<int> para;
    vector<double> result;
    para.clear();
    result.clear();
    // cout << result.size();
    bool count = 0;
    while (fin >> value)
    {
        if (value != 99999)
        {
            if (!count) {number = value; count = 1;}
            else para.push_back(value);
        }
        else
        {
            // cout << para.size() << " " << result.size() << endl;
            //Here to add AR
            if (!autoRegression(para, 5, result))
            {
                cout << "Unable to predict!!" << endl;
                count = 0;
                para.clear();
                result.clear();
                number = 0;
                continue;
            }
            // cout << result[0] << endl;
            cout << result[0] << endl;
            // cout << number << " " << para[0] << endl;
            count = 0;
            para.clear();
            result.clear();
            number = 0;
        }
    }
    if (fin.eof()) cout << "Load Finished" << endl;
    fin.close();
}