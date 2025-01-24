#pragma once
#include "gaden_common/Vector3.h"
#include "gaden_common/Logging.h"

struct Triangle
{
    gaden::Vector3 p1;
    gaden::Vector3 p2;
    gaden::Vector3 p3;
    Triangle()
    {}
    Triangle(const gaden::Vector3& p1, const gaden::Vector3& p2, const gaden::Vector3& p3)
    {
        this->p1 = p1;
        this->p2 = p2;
        this->p3 = p3;
    }
    gaden::Vector3& operator[](int i)
    {
        if (i == 0)
            return p1;
        else if (i == 1)
            return p2;
        else if (i == 2)
            return p3;
        else
        {
            GADEN_ERROR("Indexing error when accessing the gaden::Vector3s in triangle! Index must be >= 2");
            return p1;
        }
    }
};