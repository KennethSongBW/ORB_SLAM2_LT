#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

bool autoRegression(vector<int>, int, vector<double>&);
bool ARLeastSquare(vector<double>, int, vector<double> &);
bool solveLE(vector<vector<double> >, vector<double>, int);

bool autoRegression(vector<int> input, int degree, vector<double>& output)
{
    // cout << 1 << endl;
    double mean = 0.0;
    vector<double> minus;
    // vector<double> result;
    // Determine and Subtract the mean from the input series
    for (int i = 0; i < input.size(); i++) mean += input[i];
    mean /= double(input.size());
    // cout << 2 << endl;
    for (int i = 0; i < input.size(); i++) minus.push_back(input[i] - mean);
    // cout << 3 << endl;
    // cout << output.size() << "!" << endl;
    if (!ARLeastSquare(minus, degree, output))
    {
        // cout << "Unable to predict!" << endl;
        return false;
    }
    // cout << output.size()<< " " << "!" << endl;
    // for (int i = 0; i < degree; i++) output.push_back(-ar[degree][i]);
    return true;
}

bool ARLeastSquare(vector<double> input, int degree, vector<double>& output)
{
    // cout << 1 << endl;
    vector<vector<double> > mat;
    vector<double> a;
    int length = input.size();
    int hi, hj;
    // cout << 2 << endl;
    for (int i = 0; i < degree; i++)
    {
        output.push_back(0.0);
        for (int j = 0; j < degree; j++)
        {
            a.push_back(0);
        }
        mat.push_back(a);
        a.clear();
    }
    // cout << output.size() << endl;
    for (int i = degree - 1; i < length - 1; i++)
    {
        hi = i + 1;
        for (int j = 0; j < degree; j++)
        {
            hj = i - j;
            output[j] +=input[hi] * input[hj];
            for (int k = j; k < degree; k++) mat[j][k] += input[hj] * input[i - k];
        }
    }
    // cout << 4 << endl;
    for (int i = 0; i < degree; i++)
    {
        output[i] /= length - degree;
        for (int j = i; j < degree; j++)
        {
            mat[i][j] /= length - degree;
            mat[j][i] = mat[i][j];
        }
    }
    // cout << 5 << endl;
    if (!solveLE(mat, output, degree))
    {
        // cout << "Error in solve\n";
        return false;
    }
    // cout << output.size()<< " " << "!" << endl;
    return true;
}

bool solveLE(vector<vector<double> > mat, vector<double> output, int n)
{
    // cout << output.size()<< " " << "!" << endl;
    double max, h, vswap, pivot;
    int maxi;
    vector<double> mswap, hvec;
    for (int i = 0; i < n - 1; i++)
    {
        // cout << 1 << endl;
        max = abs(mat[i][i]);
        maxi = i;
        for (int j = i + 1; j < n; j++)
        {
            h = abs(mat[j][i]);
            if (h > max)
            {
                max = h;
                maxi = j;
            }
        }
        // cout << 2 << endl;
        if (maxi != i)
        {
            mswap = mat[i];
            mat[i] = mat[maxi];
            mat[maxi] = mswap;
            vswap = output[i];
            output[i] = output[maxi];
            output[maxi] = vswap;
        }
        // cout << 3 << endl;
        hvec = mat[i];
        pivot = hvec[i];
        if (abs(pivot) == 0.0)
        {
            // cout << "Error: singular\n";
            return false;
        }
        // cout << 4 << endl;
        for (int j = i + 1; j < n; j++)
        {
            double q = -mat[j][i] / pivot;
            mat[j][i] = 0.0;
            for (int k = i + 1; k < n; k++) mat[j][k] += q * hvec[k];
            output[j] += q * output[i];
        }
    }
    // cout << 5 << endl;
    output[n - 1] /= mat[n - 1][n - 1];
    // cout << 6 << endl;
    for (int i = n-2; i >= 0; i--)
    {
        hvec = mat[i];
        for (int j = n-1; j > 1; j--) output[i] -= hvec[j] * output[j];
        output[i] /= hvec[i];
    }
    // cout << 7 << endl;
    return true;
}

