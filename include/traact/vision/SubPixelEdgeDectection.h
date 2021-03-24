/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
#define TRAACTMULTI_SUBPIXELEDGEDECTECTION_H

namespace traact::vision {
    template<class T> class SubPixelEdgeDectection
    {
    public:
        virtual ~SubPixelEdgeDectection() {};
        virtual double findEdge(T* startInImage, const int positionIncrement, T threshold, const int maxDistance) = 0;
    };

    template<class T> class ThresholdSubPixelEdgeDectection
    {
    public:
        ~ThresholdSubPixelEdgeDectection() {};
        double findEdge(T* startInImage, const int positionIncrement, T threshold, const int maxDistance) {



            for(int i=0;i<maxDistance; ++i)
            {
                int imageIndex = i * positionIncrement;

                if(startInImage[imageIndex] < threshold)
                {
                    if(i == 0) return 0;

                    int prevIndex = imageIndex - positionIncrement;

                    T prevvalue = startInImage[prevIndex];
                    T value = startInImage[imageIndex];

                    double delta = static_cast<double>(prevvalue - threshold) / static_cast<double>(prevvalue - value);

                    return static_cast<double>(i)+delta;
                }
            }

            return 0;
        }
    };
}

#endif //TRAACTMULTI_SUBPIXELEDGEDECTECTION_H
