#pragma once

#include <string>
#include <vector>

class AbstractMap
{
    friend class MapFuser;

public:
    enum CellType
    {
        UNKNOWN_CELL = -1,
        FREE_CELL = 0,
        FILLED_GAP = 90,
        OBSTACLE_CELL = 100,

    };

    struct MapParams
    {
        double mapSizeMeters;
        double cellSizeMeters;
        double minimumGapSizeMetersSquared;
        unsigned int numCellsPerRowCol;
    };

private:
    std::vector<int8_t> mapMatrix;

protected:
    static MapParams params;

    virtual void reset()
    {
        std::fill(this->mapMatrix.begin(), this->mapMatrix.end(), CellType::UNKNOWN_CELL);
    }

    virtual std::vector<int8_t>::const_iterator begin() const
    {
        return mapMatrix.begin();
    }

    virtual std::vector<int8_t>::const_iterator end() const
    {
        return mapMatrix.end();
    }

    virtual int8_t operator[](const unsigned int index) const
    {
        return mapMatrix[index];
    }

    virtual void set(unsigned int y, unsigned int x, int8_t val)
    {
        mapMatrix[y * params.numCellsPerRowCol + x] = val;
    }

    virtual int8_t get(unsigned int y, unsigned int x) const
    {
        return mapMatrix[y * params.numCellsPerRowCol + x];
    }

    /*int8_t get(const AbstractMap &m, unsigned int y, unsigned int x) const
    {
        return m[y * params.numCellsPerRowCol + x];
    }*/

public:
    AbstractMap()
    {
        this->resize();
    }

    static unsigned int toIndex(double val)
    {
        val /= params.cellSizeMeters;
        val += params.numCellsPerRowCol / 2.0;
        return val;
    }

    static double toMeters(unsigned int idx)
    {
        double result = idx;
        result -= params.numCellsPerRowCol / 2.0;
        return result * params.cellSizeMeters;
    }

    static void setParams(const MapParams &_params)
    {
        params = _params;
    }

    virtual void resize()
    {
        mapMatrix.resize(params.numCellsPerRowCol * params.numCellsPerRowCol);
        this->reset();
    }
};

AbstractMap::MapParams AbstractMap::params = {0, 0, 0, 0};