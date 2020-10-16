#include "SharpIR.h"
#include "Arduino.h"

SharpIR::SharpIR(int irPin, const uint32_t sensorType, double m, double b, double k) {
		_irPin = irPin;
		_model = sensorType;
		_m = m;
		_b = b;
		_k = k;
		pinMode(_irPin, INPUT);
}

void SharpIR::sort(int a[], int n) const {
	int i = 0;
	int j = 0;
	int next = 0;
	for (i = 0; i < n; i++) {
		next = a[i];
		for (j = i - 1; j >= 0 && a[j] > next; j--) {
			a[j + 1] = a[j];
		}
		a[j + 1] = next;
	}
}

int SharpIR::getRaw() const {
	int irValue[sampleSize];
	int i = 0;
	for (i = 0; i < sampleSize; i++) {
		irValue[i] = analogRead(_irPin);
	}
	sort(irValue, sampleSize);
	return irValue[sampleSize / 2];
}

double SharpIR::distance() const {
	int irValue = getRaw();
	double distance = 0;
	if (_model == GP2Y0A21YK0F && irValue < SRVMin) {
		irValue = SRVMin;
	} else if (_model == GP2Y0A02YK0F && irValue < LRVMin) {
		irValue = LRVMin ;
	}
	distance = (_m / (irValue + _b)) - _k;
	return distance;
}
