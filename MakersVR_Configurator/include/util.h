/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef UTIL_H
#define UTIL_H

typedef struct
{
	double start, avg, diff, min, max, num, cur;
} StatValue;

void UpdateStatValue(StatValue *stat, double cur);

#endif // UTIL_H