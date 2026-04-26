


// ### 📁 4. Менеджер переключения `FilterManager.h` / `.cpp`
// Обеспечивает безопасную подмену фильтра 
// без скачков ориентации  даже в полёте.
//
// FilterManager.h 
// cpp
// 
#pragma once
#include "VibrationEstimator.h"
#include "ISensorFilter.h"
#include "MahonyFilter.h"
#include "MadgwickFilter.h"

enum class FilterType { MAHONY, MADGWICK };

class FilterManager {
public:
    FilterManager();
    void begin(float sampleFreqHz);
    
    // 🔑 Адаптивность включается/выключается одной командой
    void enableAdaptive(bool enable);
    bool isAdaptiveEnabled() const { return _adaptiveEnabled; }
    
    void update(float dt, float ax, float ay, float az, float gx, float gy, float gz, 
                float mx=0, float my=0, float mz=0);
    // Безопасное переключение (в полёте или на земле)
    void requestSwitch(FilterType type);
    FilterType getActiveType() const { return _currentType; }
    
    void getQuaternion(float &w, float &x, float &y, float &z) const;
    void getEuler(float &roll, float &pitch, float &yaw) const;
    const char* getName() const { return _activeFilter->getName(); }
    
    // Для отладки
    float getCurrentVibrationLevel() const { return _vibEstimator.getVibrationLevel(); }

private:
    MahonyFilter _mahony;
    MadgwickFilter _madgwick;
    VibrationEstimator _vibEstimator;
    ISensorFilter* _activeFilter;
    FilterType _currentType;
    volatile bool _pendingSwitch;
    FilterType _pendingType;
    bool _adaptiveEnabled = false;
};
