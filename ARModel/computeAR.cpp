#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iterator>
#include "ar.cpp"

using namespace std;

int main()
{
    const int MAX = 10;
    const int MIN = 3;
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
            //Here to perform degree computation
            vector<double> s;
            vector<vector<double> > results;
            for (int i = MIN; i < MAX; i++)
            {
                // s.push_back(0);
                // cout << 1 << endl;
                double x;
                if (!autoRegression(para, i, result, x))
                {
                    cout << "Unable to predict!!" << endl;
                    result.push_back(0);
                    results.push_back(result);
                    count = 0;
                    para.clear();
                    result.clear();
                    number = 0;
                    continue;
                }
                s.push_back(x);
                results.push_back(result);
                s.at(i-MIN) = log(s[i]) + 2 * i / double(para.size());
            }
            vector<double>::iterator small = min_element(s.begin(),s.end());
            int degree = distance(s.begin(), small);
            result = results[degree];
            cout << result[0] << " " << degree << endl;
            count = 0;
            para.clear();
            result.clear();
            number = 0;
        }
    }
    if (fin.eof()) cout << "Load Finished" << endl;
    fin.close();
}