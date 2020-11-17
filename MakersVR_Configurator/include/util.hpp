/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef UTIL_H
#define UTIL_H

#include <algorithm>

struct StatValue
{
	double start, avg, diff, min, max, num, cur;
};

/**
 * Updates the given statistical value with the new value
 */
static void UpdateStatValue(StatValue *stat, double cur)
{
	if (stat->num == 0) stat->start = cur;
	stat->min = std::min(stat->min, cur);
	stat->max = std::max(stat->max, cur);
	stat->avg = (stat->avg * stat->num + cur) / (stat->num + 1);
	stat->diff += (abs(cur - stat->avg) - stat->diff) / (stat->num + 1);
	stat->num++;
	stat->cur = cur;
}

#endif // UTIL_H