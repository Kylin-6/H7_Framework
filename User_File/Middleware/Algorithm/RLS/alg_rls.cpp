/**
 * @file alg_rls.cpp
 * @brief 定长递推最小二乘辨识
 */

#include "alg_rls.h"

#include <math.h>

static bool Covariance_Is_Positive_Definite(const double covariance[RLS_MAX_DIMENSION][RLS_MAX_DIMENSION],
                                            uint8_t dimension)
{
    double lower[RLS_MAX_DIMENSION][RLS_MAX_DIMENSION] = {};
    for (uint8_t i = 0; i < dimension; i++)
    {
        for (uint8_t j = 0; j <= i; j++)
        {
            double value = covariance[i][j];
            for (uint8_t k = 0; k < j; k++)
            {
                value -= lower[i][k] * lower[j][k];
            }
            if (!isfinite(value))
            {
                return (false);
            }
            if (i == j)
            {
                if (value <= 0.0)
                {
                    return (false);
                }
                lower[i][j] = sqrt(value);
            }
            else
            {
                lower[i][j] = value / lower[j][j];
            }
            if (!isfinite(lower[i][j]))
            {
                return (false);
            }
        }
    }
    return (true);
}

bool Class_RLS::Init(uint8_t dimension, float forgetting_factor, float initial_covariance,
                     const float *initial_parameters)
{
    if (dimension == 0 || dimension > RLS_MAX_DIMENSION ||
        !isfinite(forgetting_factor) || forgetting_factor <= 0.0f || forgetting_factor > 1.0f ||
        !isfinite(initial_covariance) || initial_covariance <= 0.0f)
    {
        return (false);
    }

    for (uint8_t i = 0; i < dimension; i++)
    {
        if (initial_parameters != NULL && !isfinite(initial_parameters[i]))
        {
            return (false);
        }
    }

    Dimension = dimension;
    Forgetting_Factor = forgetting_factor;
    Initial_Covariance = initial_covariance;
    for (uint8_t i = 0; i < RLS_MAX_DIMENSION; i++)
    {
        Initial_Parameters[i] = i < dimension && initial_parameters != NULL ? initial_parameters[i] : 0.0;
    }
    Initialized_Flag = true;
    Reset();
    return (true);
}

bool Class_RLS::Update(const float *features, float measurement)
{
    if (!Initialized_Flag || features == NULL || !isfinite(measurement))
    {
        return (false);
    }

    double feature[RLS_MAX_DIMENSION] = {};
    double covariance_feature[RLS_MAX_DIMENSION] = {};
    double new_parameters[RLS_MAX_DIMENSION] = {};
    double new_covariance[RLS_MAX_DIMENSION][RLS_MAX_DIMENSION] = {};
    double prediction = 0.0;

    for (uint8_t i = 0; i < Dimension; i++)
    {
        if (!isfinite(features[i]))
        {
            return (false);
        }
        feature[i] = features[i];
        prediction += feature[i] * Parameters[i];
        for (uint8_t j = 0; j < Dimension; j++)
        {
            covariance_feature[i] += Covariance[i][j] * features[j];
        }
        if (!isfinite(prediction) || !isfinite(covariance_feature[i]))
        {
            return (false);
        }
    }

    double denominator = Forgetting_Factor;
    for (uint8_t i = 0; i < Dimension; i++)
    {
        denominator += feature[i] * covariance_feature[i];
    }
    double residual = (double)measurement - prediction;
    if (!isfinite(denominator) || denominator <= 0.0 || !isfinite(residual))
    {
        return (false);
    }

    for (uint8_t i = 0; i < Dimension; i++)
    {
        new_parameters[i] = Parameters[i] + covariance_feature[i] * (residual / denominator);
        if (!isfinite(new_parameters[i]) || !isfinite((float)new_parameters[i]))
        {
            return (false);
        }
        for (uint8_t j = i; j < Dimension; j++)
        {
            double value = (Covariance[i][j] - covariance_feature[i] *
                            (covariance_feature[j] / denominator)) / Forgetting_Factor;
            if (!isfinite(value) || (i == j && value <= 0.0))
            {
                return (false);
            }
            new_covariance[i][j] = value;
            new_covariance[j][i] = value;
        }
    }

    if (!Covariance_Is_Positive_Definite(new_covariance, Dimension))
    {
        return (false);
    }

    for (uint8_t i = 0; i < Dimension; i++)
    {
        Parameters[i] = new_parameters[i];
        for (uint8_t j = 0; j < Dimension; j++)
        {
            Covariance[i][j] = new_covariance[i][j];
        }
    }
    return (true);
}

void Class_RLS::Reset()
{
    if (!Initialized_Flag)
    {
        return;
    }

    for (uint8_t i = 0; i < RLS_MAX_DIMENSION; i++)
    {
        Parameters[i] = Initial_Parameters[i];
        for (uint8_t j = 0; j < RLS_MAX_DIMENSION; j++)
        {
            Covariance[i][j] = i == j && i < Dimension ? Initial_Covariance : 0.0;
        }
    }
}

bool Class_RLS::Get_Parameters(float *out, uint8_t capacity) const
{
    if (!Initialized_Flag || out == NULL || capacity < Dimension)
    {
        return (false);
    }
    for (uint8_t i = 0; i < Dimension; i++)
    {
        out[i] = (float)Parameters[i];
    }
    return (true);
}

float Class_RLS::Get_Parameter(uint8_t index) const
{
    if (!Initialized_Flag || index >= Dimension)
    {
        return (0.0f);
    }
    return ((float)Parameters[index]);
}

uint8_t Class_RLS::Get_Dimension() const
{
    return (Dimension);
}

bool Class_RLS::Get_Initialized_Flag() const
{
    return (Initialized_Flag);
}
