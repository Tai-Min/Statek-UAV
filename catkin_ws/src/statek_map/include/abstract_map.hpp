#pragma once

#include <string>
#include <vector>

class AbstractMap
{
    friend class MapFuser;

protected:
    enum CellType
    {
        UNKNOWN_CELL = -1,
        FREE_CELL = 0,
        OBSTACLE_CELL = 1
    };

public:
    struct MapParams
    {
        double mapSizeMeters;
        double cellSizeMeters;
        double minimumGapSizeMeters;
        unsigned int numCellsPerRowCol;
    };

private:
    std::vector<int8_t> mapMatrix;

protected:
    static MapParams params;

    static unsigned int toIndex(double val)
    {
        val /= params.cellSizeMeters;
        val += params.numCellsPerRowCol;
        return val;
    }

    static double toMeters(unsigned int idx)
    {
        double result = idx;
        result -= params.numCellsPerRowCol;
        return result * params.cellSizeMeters;
    }

    virtual void reset()
    {
        std::fill(this->mapMatrix.begin(), this->mapMatrix.end(), -1);
    }

    std::vector<int8_t>::const_iterator begin() const
    {
        return mapMatrix.begin();
    }

    std::vector<int8_t>::const_iterator end() const
    {
        return mapMatrix.end();
    }

    int8_t operator[](const unsigned int index) const
    {
        return mapMatrix[index];
    }

    void set(unsigned int y, unsigned int x, int8_t val)
    {
        mapMatrix[y * params.numCellsPerRowCol + x] = val;
    }

    int8_t get(unsigned int y, unsigned int x) const
    {
        return mapMatrix[y * params.numCellsPerRowCol + x];
    }

    int8_t get(const AbstractMap &m, unsigned int y, unsigned int x) const
    {
        return m[y * params.numCellsPerRowCol + x];
    }

public:
    AbstractMap()
    {
        mapMatrix.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
    }

    static void setParams(const MapParams &_params)
    {
        params = _params;
    }

    void resize()
    {
        mapMatrix.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
        reset();
    }
};

AbstractMap::MapParams AbstractMap::params = {0, 0, 0, 0};