#ifndef __XFORM_H__
#define __XFORM_H__

/**
 * This contains some often used mathematical definitions and functions.
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#include <math.h>
#include <stdlib.h>
#include <cstdio>

namespace xform {

inline double sec(const double a) { return 1 / cos(a); }

inline double cosec(const double a) { return 1 / sin(a); }

inline double Abs(double a) {
    if (a >= 0.0)
        return a;
    else
        return -a;
}

/** @name constants for some often used angles */
///@{
/** constant for a half circle*/
const double pi = 3.1415926535897932384626433832795;
/** constant for a full circle*/
const double pi2 = 2.0 * 3.1415926535897932384626433832795;
/** constant for three quater circle*/
const double pi3_2 = 1.5 * 3.1415926535897932384626433832795;
/** constant for a quarter circle*/
const double pi_2 = 0.5 * 3.1415926535897932384626433832795;
/** constant for a 1 degree*/
const double pi_180 = 3.1415926535897932384626433832795 / 180;
/** constant for a 1/8 circle*/
const double pi_4 = 3.1415926535897932384626433832795 * 0.25;
/** constant for a 3/8 circle*/
const double pi3_4 = 3.1415926535897932384626433832795 * 0.75;
///@}

const double pi180_ = 180 / 3.1415926535897932384626433832795;

const double Million = 1000000.0;

const double two_pi = 2.0 / 3.1415926535897932384626433832795;
/** @name constant for bayesian filter */
///@{
/** constant for e*/
const double e = 2.71828182845902353602874713527;
///@}
/**
 * Converts angle from rad to long in microrad.
 * Robot uses long microrad for joint angles.
 * \param angle coded in rad
 * \return angle coded in microrad
 */
inline long toMicroRad(double angle) { return long(angle * 1000000.0); }

/**
 * Converts angle long in microrad to rad.
 * \param microrad angle coded in microrad
 * \return angle coded in rad
 */
inline double fromMicroRad(long microrad) { return ((double)microrad) / 1000000.0; }

/**
 * Converts angle from rad to degrees.
 * \param angle code in rad
 * \return angle coded in degrees
 */
inline double toDegrees(double angle) { return angle * 180.0 / pi; }

/** Converts angle from degrees to rad.
 * \param degrees angle coded in degrees
 * \return angle coded in rad
 */
inline double fromDegrees(double degrees) { return degrees * pi_180; }

/**
 * reduce angle to [-pi..+pi]
 * \param data angle coded in rad
 * \return normalized angle coded in rad
 */
inline double normalize(double data) {
    if (data <= pi && data >= -pi)
        return data;
    double ndata = data - ((int)(data / pi2)) * pi2;
    if (ndata > pi) {
        ndata -= pi2;
    } else if (ndata < -pi) {
        ndata += pi2;
    }
    return ndata;
}

inline double unnormalize(double data) {
    if (data >= 0.0 && data <= pi)
        return data;
    if (data < 0.0 && data > -pi)
        return 2 * pi + data;
}

/**
 * The function returns a random number in the range of [0..1].
 * @return The random number.
 */
inline double random() { return double(rand()) / RAND_MAX; }

/**
 * The function returns a random integer number in the range of [0..n-1].
 * @param n the number of possible return values (0 ... n-1)
 * @return The random number.
 */
inline int random(int n) { return (int)(random() * n * 0.999999); }

inline double fmax(double a, double b) { return fabs(a) > fabs(b) ? a : b; }

/**
 * Round floating point value to nearest integer.
 *
 * \note Not tested (or even useful?!?) on MIPS platform!
 * \see http://www.agner.org/assem/
 */
/*inline int fast_round(float val)
 {
 double temp = val + 6755399441055744.0;
 return (int)*((__int64*)&temp);
 }*/

/**
 * Magical fast floor function for floats.
 *
 * \note Not tested (or even useful?!?) on MIPS platform!
 * \see http://www.agner.org/assem/
 */
/*inline int fast_floor(float val)
 {
 double temp = val + 6755399441055744.0;
 float  diff = (float)(val - (int)*((__int64*)&temp));
 return (int)*((__int64*)&temp) - (*(int*)&diff < 0);
 }*/

/**
 * A template class to represent ranges. It also defines the 13 Allen relations
 */
template <class T>
class Range {
   public:
    T min, max; /**< The limits of the range. */

    /**
     * Constructor.
     * Defines an empty range.
     */
    Range() { min = max = 0; }

    /**
     * Constructor.
     * @param min The minimum of the range.
     * @param max The maximum of the range.
     */
    Range(T min, T max) {
        Range::min = min;
        Range::max = max;
    }

    /**
     * The function enlarges the range so that a certain value will be part of it.
     * @param t The value that will be part of the range.
     * @return A reference to the range.
     */
    Range<T> &add(T t) {
        if (min > t)
            min = t;
        if (max < t)
            max = t;
        return *this;
    }

    /**
     * The function enlarges the range so that the resulting range also contains another one.
     * @param r The range that also will be part of the range.
     * @return A reference to the range.
     */
    Range<T> &add(const Range<T> &r) {
        add(r.min);
        add(r.max);
        return *this;
    }

    /**
     * The function checks whether a certain value is in the range.
     * Note that the function is able to handle circular range, i.e. max < min.
     * @param t The value.
     * @return Is the value inside the range?
     */
    bool isInside(T t) const { return min <= max ? t >= min && t <= max : t >= min || t <= max; }

    /**
     * The function limits a certain value to the range.
     * Note that the function is not able to handle circular range, i.e. max < min.
     * @param t The value that will be "clipped" to the range.
     * @return The limited value.
     */
    T limit(T t) const { return t < min ? min : t > max ? max : t; }  // sets a limit for a Range

    /**
     * The function limits another range to this range.
     * Note that the function is able to handle circular range, i.e. max < min.
     * @param r The range that will be "clipped" to this range.
     * @return The limited value.
     */
    Range<T> limit(const Range<T> &r) const {
        return Range<T>(limit(r.min), limit(r.max));
    }  // sets the limit of a Range

    /**
     * The function returns the size of the range.
     * @return The difference between the lower limit and the higher limit.
     */
    T getSize() const { return max - min; }

    /**
     * The function returns the center of the range.
     * @return The center.
     */
    T getCenter() const { return (max + min) / 2; }

    //!@name The 13 Allen relations
    //!@{
    bool operator==(const Range<T> &r) const { return min == r.min && max == r.max; }

    bool operator<(const Range<T> &r) const { return max < r.min; }

    bool operator>(const Range<T> &r) const { return min > r.max; }

    bool meets(const Range<T> &r) const { return max == r.min; }

    bool metBy(const Range<T> &r) const { return min == r.max; }

    bool overlaps(const Range<T> &r) const { return min < r.min && max < r.max && max > r.min; }

    bool overlappedBy(const Range<T> &r) const { return min > r.min && max > r.max && min < r.max; }

    bool starts(const Range<T> &r) const { return min == r.min && max < r.max; }

    bool startedBy(const Range<T> &r) const { return min == r.min && max > r.max; }

    bool finishes(const Range<T> &r) const { return max == r.max && min > r.min; }

    bool finishedBy(const Range<T> &r) const { return max == r.max && min < r.min; }

    bool during(const Range<T> &r) const { return min > r.min && max < r.max; }

    bool contains(const Range<T> &r) const { return min < r.min && max > r.max; }
    //!@}
};

/** This class represents a 2-vector */
template <class V>
class Vector2 {
   public:
    /** The vector values */
    V x, y;

    /** Default constructor. */
    Vector2<V>() : x(0), y(0) {}

    /** Default constructor. */
    Vector2<V>(V x, V y) : x(x), y(y) {}

    /** Assignment operator
     *\param other The other vector that is assigned to this one
     *\return A reference to this object after the assignment.
     */
    Vector2<V> &operator=(const Vector2<V> &other) {
        x = other.x;
        y = other.y;
        return *this;
    }

    /** Copy constructor
     *\param other The other vector that is copied to this one
     */
    Vector2<V>(const Vector2<V> &other) { *this = other; }

    /** Addition of another vector to this one.
     *\param other The other vector that will be added to this one
     *\return A reference to this object after the calculation.
     */
    Vector2<V> &operator+=(const Vector2<V> &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    /** Substraction of this vector from another one.
     *\param other The other vector this one will be substracted from
     *\return A reference to this object after the calculation.
     */
    Vector2<V> &operator-=(const Vector2<V> &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    /** Multiplication of this vector by a factor.
     *\param factor The factor this vector is multiplied by
     *\return A reference to this object after the calculation.
     */
    Vector2<V> &operator*=(const V &factor) {
        x *= factor;
        y *= factor;
        return *this;
    }

    /** Division of this vector by a factor.
     *\param factor The factor this vector is divided by
     *\return A reference to this object after the calculation.
     */
    Vector2<V> &operator/=(const V &factor) {
        if (factor == 0)
            return *this;
        x /= factor;
        y /= factor;
        return *this;
    }

    /** Addition of another vector to this one.
     *\param other The other vector that will be added to this one
     *\return A new object that contains the result of the calculation.
     */
    Vector2<V> operator+(const Vector2<V> &other) const { return Vector2<V>(*this) += other; }

    /** Subtraction of another vector to this one.
     *\param other The other vector that will be added to this one
     *\return A new object that contains the result of the calculation.
     */
    Vector2<V> operator-(const Vector2<V> &other) const { return Vector2<V>(*this) -= other; }

    /** Negation of this vector.
     *\return A new object that contains the result of the calculation.
     */
    Vector2<V> operator-() const { return Vector2<V>() -= *this; }

    /** Inner product of this vector and another one.
     *\param other The other vector this one will be multiplied by
     *\return The inner product.
     */
    V operator*(const Vector2<V> &other) const { return x * other.x + y * other.y; }

    /** Multiplication of this vector by a factor.
     *\param factor The factor this vector is multiplied by
     *\return A new object that contains the result of the calculation.
     */
    Vector2<V> operator*(const V &factor) const { return Vector2<V>(*this) *= factor; }

    /** Division of this vector by a factor.
     *
     *\param factor The factor this vector is divided by
     *\return A new object that contains the result of the calculation.
     */
    Vector2<V> operator/(const V &factor) const { return Vector2<V>(*this) /= factor; }

    /** Comparison of another vector with this one.
     *\param other The other vector that will be compared to this one
     *\return Whether the two vectors are equal.
     */
    bool operator==(const Vector2<V> &other) const { return (x == other.x && y == other.y); }

    /** Comparison of another vector with this one.
     *\param other The other vector that will be compared to this one.
     *\return Whether the two vectors are unequal.
     */
    bool operator!=(const Vector2<V> &other) const { return !(*this == other); }

    bool operator>(const Vector2<V> &other) const {
        if (x > other.x && y > other.y)
            return 1;
        else
            return 0;
    }

    /** compare a vector with another vector
     */
    bool operator<(const Vector2<V> &other) const {
        if (x < other.x && y < other.y)
            return 1;
        else
            return 0;
    }

    bool operator>=(const Vector2<V> &other) const {
        if (x >= other.x && y >= other.y)
            return 1;
        else
            return 0;
    }

    /** compare a vector with another vector
     */
    bool operator<=(const Vector2<V> &other) const {
        if (x <= other.x && y <= other.y)
            return 1;
        else
            return 0;
    }

    /** Calculation of the square of the length of this vector.
     This value can save a 'sqrt' calculation
     *\return The square of the length.
     */
    V absSquare() const { return (V)(((double)x) * x + ((double)y) * y); }

    /** Calculation of the length of this vector.
     *\return The length.
     */
    V abs() const { return (V)sqrt(((double)x) * x + ((double)y) * y); }

    /** normalize this vector.
     *\param len The length, the vector should be normalized to, default=1.
     *\return the normalized vector.
     */
    Vector2<V> normalize(V len) {
        if (abs() == 0)
            return *this;
        return *this = (*this * len) / abs();
    }

    /** normalize this vector.
     *\return the normalized vector.
     */
    Vector2<V> normalize() {
        if (abs() == 0)
            return *this;
        return *this /= abs();
    }

    /** transpose this vector.
     *\return the transposed vector.
     */
    Vector2<V> transpose() {
        V buffer = x;
        x        = y;
        y        = buffer;
        return *this;
    }

    /** the vector is rotated left by 90 degrees.
     *\return the rotated vector.
     */
    Vector2<V> rotateLeft() {
        V buffer = -y;
        y        = x;
        x        = buffer;
        return *this;
    }

    /** the vector is rotated right by 90 degrees.
     *\return the rotated vector.
     */
    Vector2<V> rotateRight() {
        V buffer = -x;
        x        = y;
        y        = buffer;
        return *this;
    }

    /**
     * array-like member access.
     * \param i index of coordinate
     * \return reference to x or y
     */
    V &operator[](int i) { return (&x)[i]; }

    /** Calculation of the angle of this vector */
    double angle() const { return atan2((double)y, (double)x); }
};

/** representation for 2D Transformation and Position (Location + Orientation)*/
class Pose2D {
   public:
    /** Rotation as an angle*/
    double rotation;

    /** translation as an vector2*/
    Vector2<double> translation;

    /** noargs-constructor*/
    Pose2D() : rotation(0), translation(0, 0) {}

    /** constructor from rotation and translation
     * \param rot rotation (double)
     * \param trans translation (Vector2)
     */
    /* vc++ bug, translation = trans is workaround */
    Pose2D(const double rot, const Vector2<double> &trans) : rotation(rot), translation(trans) {}

    /** constructor from rotation and translation
     * \param rot rotation (double)
     * \param x translation.x (double)
     * \param y translation.y (double)
     */
    Pose2D(const double rot, const double x, const double y) : rotation(rot), translation(x, y) {}

    /** constructor from rotation
     * \param rot rotation (double)
     */
    /* vc++ bug, translation = trans is workaround */
    Pose2D(const double rot) : rotation(rot), translation(0, 0) {}

    /** constructor from translation
     * \param trans translation (Vector2)
     */
    /* vc++ bug, translation = trans is workaround */
    Pose2D(const Vector2<double> &trans) : rotation(0), translation(trans) {}

    /** constructor from translation
     * \param trans translation (Vector2)
     */
    /* vc++ bug, translation = trans is workaround */
    Pose2D(const Vector2<int> &trans) : rotation(0), translation(trans.x, trans.y) {}

    /** constructor from two translation values
     * \param x translation x component
     * \param y translation y component
     */
    Pose2D(const double x, const double y) : rotation(0), translation(x, y) {}

    /** get the Angle
     * @return Angle the Angle which defines the rotation
     */
    inline double getAngle() const { return rotation; }

    /** set rotation from Angle
     * @return the new Pose2D
     */
    inline Pose2D fromAngle(const double a) {
        rotation = a;
        return *this;
    }

    /** get the cos of the angle
     * @return the cos of the angle
     */
    inline double getCos() const { return cos(rotation); }

    /** get the sin of the angle
     * @return the sin of the angle
     */
    inline double getSin() const { return sin(rotation); }

    /** Assignment operator
     *\param other The other Pose2D that is assigned to this one
     *\return A reference to this object after the assignment.
     */
    Pose2D &operator=(const Pose2D &other) {
        rotation    = other.rotation;
        translation = other.translation;
        return *this;
    }

    /** Copy constructor
     *\param other The other vector that is copied to this one
     */
    Pose2D(const Pose2D &other) { *this = other; }

    /** Multiplication of a Vector2 with this Pose2D
     *\param point The Vector2 that will be multiplicated with this Pose2D
     *\return The resulting Vector2
     */

    Vector2<double> operator*(const Vector2<double> &point) const {
        double s = sin(rotation);
        double c = cos(rotation);
        return (Vector2<double>(point.x * c - point.y * s, point.x * s + point.y * c) + translation);
    }

    /** Comparison of another pose with this one.
     *\param other The other pose that will be compared to this one
     *\return Whether the two poses are equal.
     */
    bool operator==(const Pose2D &other) const {
        return ((translation == other.translation) && (rotation == other.rotation));
    }

    /** Comparison of another pose with this one.
     *\param other The other pose that will be compared to this one
     *\return Whether the two poses are unequal.
     */
    bool operator!=(const Pose2D &other) const { return !(*this == other); }

    /**Concatenation of this pose with another pose.
     *\param other The other pose that will be concatenated to this one.
     *\return A reference to this pose after concatenation.
     */
    Pose2D &operator+=(const Pose2D &other) {
        translation = *this * other.translation;
        rotation += other.rotation;
        rotation = normalize(rotation);
        return *this;
    }

    /**A concatenation of this pose and another pose.
     *\param other The other pose that will be concatenated to this one.
     *\return The resulting pose.
     */
    Pose2D operator+(const Pose2D &other) const { return Pose2D(*this) += other; }

    /**Subtracts a difference pose from this one to get the source pose. So if A+B=C is the addition/concatenation, this
     *calculates C-B=A. \param diff The difference Pose2D that shall be subtracted. \return The resulting source pose.
     *Adding diff to it again would give the this pose back.
     */
    Pose2D minusDiff(const Pose2D &diff) const {
        double rot = rotation - diff.rotation;
        double s   = sin(rot);
        double c   = cos(rot);
        return Pose2D(rot, translation.x - c * diff.translation.x + s * diff.translation.y,
                      translation.y - c * diff.translation.y - s * diff.translation.x);
    }

    /**Difference of this pose relative to another pose. So if A+B=C is the addition/concatenation, this calculates
     *C-A=B. \param other The other pose that will be used as origin for the new pose. \return A reference to this pose
     *after calculating the difference.
     */
    Pose2D &operator-=(const Pose2D &other) {
        translation -= other.translation;
        Pose2D p(-other.rotation);
        return *this = p + *this;
    }

    /**Difference of this pose relative to another pose.
     *\param other The other pose that will be used as origin for the new pose.
     *\return The resulting pose.
     */
    Pose2D operator-(const Pose2D &other) const { return Pose2D(*this) -= other; }

    /**Concatenation of this pose with another pose
     *\param other The other pose that will be concatenated to this one.
     *\return A reference to this pose after concatenation
     */

    /**Difference of this pose relative to another pose (0,0,0) .
     *\param other The other pose that will be used as origin for the new pose.
     *\return A reference to this pose after calculating the difference.
     */
    Pose2D &operator-(void) {
        translation = -translation;
        Pose2D p(-rotation);
        return *this;
    }

    Pose2D &conc(const Pose2D &other) { return *this += other; }

    /**Translate this pose by a translation vector
     *\param trans Vector to translate with
     *\return A reference to this pose after translation
     */
    Pose2D &translate(const Vector2<double> &trans) {
        translation = *this * trans;
        return *this;
    }

    /**Translate this pose by a translation vector
     *\param x x component of vector to translate with
     *\param y y component of vector to translate with
     *\return A reference to this pose after translation
     */
    Pose2D &translate(const double x, const double y) {
        translation = *this * Vector2<double>(x, y);
        return *this;
    }

    /**Rotate this pose by a rotation
     *\param angle Angle to rotate.
     *\return A reference to this pose after rotation
     */
    Pose2D &rotate(const double angle) {
        rotation += angle;
        return *this;
    }

    /**
     * The function creates a random pose.
     * @param x The range for x-values of the pose.
     * @param y The range for y-values of the pose.
     * @param angle The range for the rotation of the pose.
     */
    static Pose2D random(const Range<double> &x, const Range<double> &y,
                         const Range<double> &angle) {  // angle should even work in wrap around case!
        return Pose2D(
            xform::random() * (angle.max - angle.min) + angle.min,
            Vector2<double>(xform::random() * (x.max - x.min) + x.min, xform::random() * (y.max - y.min) + y.min));
    }

    bool operator>(const Pose2D &other) const {
        if ((rotation > other.rotation) && (translation > other.translation))
            return 1;
        else
            return 0;
    }

    bool operator<(const Pose2D &other) const {
        if ((rotation < other.rotation) && (translation < other.translation))
            return 1;
        else
            return 0;
    }

    bool operator>=(const Pose2D &other) const {
        if ((rotation >= other.rotation) && (translation >= other.translation))
            return 1;
        else
            return 0;
    }

    /** compare a vector with another vector
     */
    bool operator<=(const Pose2D &other) const {
        if ((rotation <= other.rotation) && (translation <= other.translation))
            return 1;
        else
            return 0;
    }

    /**
     * The function defines the operator []
     */
    double operator[](int i) {
        if (i == 2)
            return rotation;
        else
            return translation[i];
    }
};

/** This class represents a 2x2-matrix */
template <class V>
class Matrix2x2 {
   public:
    /** The columns of the matrix */
    Vector2<V> c[2];

    /** Default constructor. */
    Matrix2x2<V>() {
        c[0] = Vector2<V>(1, 0);
        c[1] = Vector2<V>(0, 1);
    }

    //! Constructor
    /*!
     \param c0 the first column of the matrix.
     \param c1 the second column of the matrix.
     */
    Matrix2x2<V>(const Vector2<V> &c0, const Vector2<V> &c1) {
        c[0] = c0;
        c[1] = c1;
    }

    //! Assignment operator
    /*!
     \param other The other matrix that is assigned to this one
     \return A reference to this object after the assignment.
     */
    Matrix2x2<V> &operator=(const Matrix2x2<V> &other) {
        c[0] = other.c[0];
        c[1] = other.c[1];
        return *this;
    }

    //! Copy constructor
    /*!
     \param other The other matrix that is copied to this one
     */
    Matrix2x2<V>(const Matrix2x2<V> &other) { *this = other; }

    /**
     * Array-like member access.
     * \param  i index
     * \return reference to column
     */
    Vector2<V> &operator[](int i) { return c[i]; }

    //! Multiplication of this matrix by vector.
    /*!
     \param vector The vector this one is multiplied by
     \return A reference to a new vector containing the result
     of the calculation.
     */
    Vector2<V> operator*(const Vector2<V> &vector) const { return (c[0] * vector.x + c[1] * vector.y); }

    //! Multiplication of this matrix by another matrix.
    /*!
     \param other The other matrix this one is multiplied by
     \return An object containing the result of the calculation.
     */
    Matrix2x2<V> operator*(const Matrix2x2<V> &other) const {
        Matrix2x2<V> returnMatrix;
        returnMatrix.c[0].x = c[0].x * other.c[0].x + c[1].x * other.c[0].y;
        returnMatrix.c[0].y = c[0].y * other.c[0].x + c[1].y * other.c[0].y;
        returnMatrix.c[1].x = c[0].x * other.c[1].x + c[1].x * other.c[1].y;
        returnMatrix.c[1].y = c[0].y * other.c[1].x + c[1].y * other.c[1].y;
        return returnMatrix;
    }

    //! Multiplication of this matrix by another matrix.
    /*!
     \param other The other matrix this one is multiplied by
     \return A reference this object after the calculation.
     */
    Matrix2x2<V> operator*=(const Matrix2x2<V> &other) { return *this = *this * other; }

    //! Multiplication of this matrix by a factor.
    /*!
     \param factor The factor this matrix is multiplied by
     \return A reference to this object after the calculation.
     */
    Matrix2x2<V> &operator*=(const V &factor) {
        c[0] *= factor;
        c[1] *= factor;
        return *this;
    }

    //! Division of this matrix by a factor.
    /*!
     \param factor The factor this matrix is divided by
     \return A reference to this object after the calculation.
     */
    Matrix2x2<V> &operator/=(const V &factor) { return *this *= 1 / factor; }

    //! Multiplication of this matrix by a factor.
    /*!
     \param factor The factor this matrix is multiplied by
     \return A new object that contains the result of the calculation.
     */
    Matrix2x2<V> operator*(const V &factor) const { return Matrix2x2<V>(*this) *= factor; }

    //! Division of this matrix by a factor.
    /*!
     \param factor The factor this matrix is divided by
     \return A new object that contains the result of the calculation.
     */
    Matrix2x2<V> operator/(const V &factor) const { return Matrix2x2<V>(*this) /= factor; }

    // by Kai
    Matrix2x2<V> operator+(const Matrix2x2<V> &other) const {
        return Matrix2x2<V>(Vector2<V>(c[0].x + other.c[0].x, c[0].y + other.c[0].y),
                            Vector2<V>(c[1].x + other.c[1].x, c[1].y + other.c[1].y));
    }

    // by Kai
    Matrix2x2<V> operator-(const Matrix2x2<V> &other) const {
        return Matrix2x2<V>(Vector2<V>(c[0].x - other.c[0].x, c[0].y - other.c[0].y),
                            Vector2<V>(c[1].x - other.c[1].x, c[1].y - other.c[1].y));
    }

    // by Kai
    Matrix2x2<V> invert() {
        double globFactor;
        if ((c[1].y * c[0].x - c[0].y * c[1].x) != 0.0)
            globFactor = 1 / (c[1].y * c[0].x - c[0].y * c[1].x);
        else
            globFactor = 0.00000000000000001;
        return Matrix2x2<V>(Vector2<V>(globFactor * c[1].y, -globFactor * c[0].y),
                            Vector2<V>(-globFactor * c[1].x, globFactor * c[0].x));
    }

    //! Comparison of another matrix with this one.
    /*!
     \param other The other matrix that will be compared to this one
     \return Whether the two matrices are equal.
     */
    bool operator==(const Matrix2x2<V> &other) const { return (c[0] == other.c[0] && c[1] == other.c[1]); }

    //! Comparison of another matrix with this one.
    /*!
     \param other The other matrix that will be compared to this one
     \return Whether the two matrixs are unequal.
     */
    bool operator!=(const Matrix2x2<V> &other) const { return !(*this == other); }

    /*! Transpose the matrix
     \return A new object containing transposed matrix
     */
    Matrix2x2<V> transpose() const { return Matrix2x2<V>(Vector2<V>(c[0].x, c[1].x), Vector2<V>(c[0].y, c[1].y)); }

    //! Calculation of the determinant of this matrix.
    /*!
     \return The determinant.
     */
    V det() const { return c[0].x * c[1].y - c[1].x * c[0].y; }
};

/** This class represents a 3-vector */
template <class V>
class Vector3 {
   public:
    /** The vector values */
    V x, y, z;

    /** Default constructor 4 gcc. */
    Vector3<V>() : x(0), y(0), z(0) {}

    /** Default constructor. */
    Vector3<V>(V x, V y, V z) : x(x), y(y), z(z) {}

    /** Assignment operator
     *\param other The other vector that is assigned to this one
     *\return A reference to this object after the assignment.
     */
    Vector3<V> &operator=(const Vector3<V> &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    /** Copy constructor
     *\param other The other vector that is copied to this one
     */
    Vector3<V>(const Vector3<V> &other) { *this = other; }

    /** Addition of another vector to this one.
     *\param other The other vector that will be added to this one
     *\return A reference to this object after the calculation.
     */
    Vector3<V> &operator+=(const Vector3<V> &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    /** Substraction of this vector from another one.
     *\param other The other vector this one will be substracted from
     *\return A reference to this object after the calculation.
     */
    Vector3<V> &operator-=(const Vector3<V> &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    /** Multiplication of this vector by a factor.
     *\param factor The factor this vector is multiplied by
     *\return A reference to this object after the calculation.
     */
    Vector3<V> &operator*=(const V &factor) {
        x *= factor;
        y *= factor;
        z *= factor;
        return *this;
    }

    /** Division of this vector by a factor.
     *\param factor The factor this vector is divided by
     *\return A reference to this object after the calculation.
     */
    Vector3<V> &operator/=(const V &factor) {
        if (factor == 0)
            return *this;
        x /= factor;
        y /= factor;
        z /= factor;
        return *this;
    }

    /** Addition of another vector to this one.
     *\param other The other vector that will be added to this one
     *\return A new object that contains the result of the calculation.
     */
    Vector3<V> operator+(const Vector3<V> &other) const { return Vector3<V>(*this) += other; }

    /** Subtraction of another vector to this one.
     *\param other The other vector that will be added to this one
     *\return A new object that contains the result of the calculation.
     */
    Vector3<V> operator-(const Vector3<V> &other) const { return Vector3<V>(*this) -= other; }

    /** Inner product of this vector and another one.
     *\param other The other vector this one will be multiplied by
     *\return The inner product.
     */
    V operator*(const Vector3<V> &other) const { return (x * other.x + y * other.y + z * other.z); }

    /** Multiplication of this vector by a factor.
     *\param factor The factor this vector is multiplied by
     *\return A new object that contains the result of the calculation.
     */
    Vector3<V> operator*(const V &factor) const { return Vector3<V>(*this) *= factor; }

    /** Division of this vector by a factor.
     *
     *\param factor The factor this vector is divided by
     *\return A new object that contains the result of the calculation.
     */
    Vector3<V> operator/(const V &factor) const { return Vector3<V>(*this) /= factor; }

    /** compare a vector with another vector
     */
    bool operator>(const Vector3<V> &other) const {
        if (x > other.x && y > other.y && z > other.z)
            return 1;
        else
            return 0;
    }

    /** compare a vector with another vector
     */
    bool operator<(const Vector3<V> &other) const {
        if (x < other.x && y < other.y && z < other.z)
            return 1;
        else
            return 0;
    }

    bool operator>=(const Vector3<V> &other) const {
        if (x >= other.x && y >= other.y && z >= other.z)
            return 1;
        else
            return 0;
    }

    /** compare a vector with another vector
     */
    bool operator<=(const Vector3<V> &other) const {
        if (x <= other.x && y <= other.y && z <= other.z)
            return 1;
        else
            return 0;
    }

    /** Comparison of another vector with this one.
     *\param other The other vector that will be compared to this one
     *\return Whether the two vectors are equal.
     */
    bool operator==(const Vector3<V> &other) const { return (x == other.x && y == other.y && z == other.z); }

    /** Comparison of another vector with this one.
     *\param other The other vector that will be compared to this one
     *\return Whether the two vectors are unequal.
     */
    bool operator!=(const Vector3<V> &other) const { return !(*this == other); }

    /**
     * array-like member access.
     * \param i index of coordinate
     * \return reference to x, y or z
     */
    V &operator[](int i) { return (&x)[i]; }

    /** Calculation of the square of the length of this vector.
     This value can save a 'sqrt' calculation
     *\return The square of the length.
     */
    V absSquare() const { return (V)(double((*this) * (*this))); }

    /** Calculation of the length of this vector.
     *\return The length.
     */
    V abs() const { return (V)sqrt(double((*this) * (*this))); }

    /** Crossproduct of this vector and another vector.
     *\param other The factor this vector is multiplied with.
     *\return A new object that contains the result of the calculation.
     */
    Vector3<V> operator^(const Vector3<V> &other) {
        return Vector3<V>(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
    }

    /** Crossproduct of this vector and another vector.
     *\param other The factor this vector is multiplied with.
     *\return A reference to this object after the calculation.
     */
    Vector3<V> &operator^=(const Vector3<V> &other) {
        *this = *this ^ other;
        return *this;
    }

    /** normalize this vector.
     *\param len The length, the vector should be normalized to, default=1.
     *\return the normalized vector.
     */
    Vector3<V> normalize(V len) {
        if (abs() == 0)
            return *this;
        return *this = (*this * len) / abs();
    }

    /** normalize this vector.
     *\return the normalized vector.
     */
    Vector3<V> normalize() {
        if (abs() == 0)
            return *this;
        return *this /= abs();
    }
};

/**
 * This class represents a 3x3-matrix
 *
 */
template <class V>
class Matrix3x3 {
   public:
    /**
     * The columns of the matrix
     */
    Vector3<V> c[3];

    /**
     * Default constructor.
     */
    Matrix3x3<V>() {
        c[0] = Vector3<V>(1, 0, 0);
        c[1] = Vector3<V>(0, 1, 0);
        c[2] = Vector3<V>(0, 0, 1);
    }

    /**
     * Constructor.
     *
     * \param  c0  the first column of the matrix.
     * \param  c1  the second column of the matrix.
     * \param  c2  the third column of the matrix.
     */
    Matrix3x3<V>(const Vector3<V> &c0, const Vector3<V> &c1, const Vector3<V> &c2) {
        c[0] = c0;
        c[1] = c1;
        c[2] = c2;
    }

    /**
     * Assignment operator.
     *
     * \param  other   The other matrix that is assigned to this one
     * \return         A reference to this object after the assignment.
     */
    Matrix3x3<V> &operator=(const Matrix3x3<V> &other) {
        c[0] = other.c[0];
        c[1] = other.c[1];
        c[2] = other.c[2];
        return *this;
    }

    /**
     * Copy constructor.
     *
     * \param other The other matrix that is copied to this one
     */
    Matrix3x3<V>(const Matrix3x3<V> &other) { *this = other; }

    /**
     * Multiplication of this matrix by vector.
     *
     * \param  vector  The vector this one is multiplied by
     * \return         A new vector containing the result
     *                 of the calculation.
     */
    Vector3<V> operator*(const Vector3<V> &vector) const {
        return (c[0] * vector.x + c[1] * vector.y + c[2] * vector.z);
    }

    /**
     * Multiplication of this matrix by another matrix.
     *
     * \param  other  The other matrix this one is multiplied by
     * \return        A new matrix containing the result
     *                of the calculation.
     */
    Matrix3x3<V> operator*(const Matrix3x3<V> &other) const {
        return Matrix3x3<V>((*this) * other.c[0], (*this) * other.c[1], (*this) * other.c[2]);
    }

    /**
     * Multiplication of this matrix by another matrix.
     *
     * \param  other  The other matrix this one is multiplied by
     * \return        A reference this object after the calculation.
     */
    Matrix3x3<V> &operator*=(const Matrix3x3<V> &other) { return *this = *this * other; }

    /**
     * Multiplication of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is multiplied by
     * \return         A reference to this object after the calculation.
     */
    Matrix3x3<V> &operator*=(const V &factor) {
        c[0] *= factor;
        c[1] *= factor;
        c[2] *= factor;
        return *this;
    }

    /**
     * Division of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is divided by
     * \return         A reference to this object after the calculation.
     */
    Matrix3x3<V> &operator/=(const V &factor) {
        *this *= 1 / factor;
        return *this;
    }

    /**
     * Multiplication of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is multiplied by
     * \return         A new object that contains the result of the calculation.
     */
    Matrix3x3<V> operator*(const V &factor) const { return Matrix3x3<V>(*this) *= factor; }

    /**
     * Division of this matrix by a factor.
     *
     * \param  factor  The factor this matrix is divided by
     * \return         A new object that contains the result of the calculation.
     */
    Matrix3x3<V> operator/(const V &factor) const { return Matrix3x3<V>(*this) /= factor; }

    /**
     * Comparison of another matrix with this one.
     *
     * \param  other  The other matrix that will be compared to this one
     * \return        Whether the two matrices are equal.
     */
    bool operator==(const Matrix3x3<V> &other) const {
        return (c[0] == other.c[0] && c[1] == other.c[1] && c[2] == other.c[2]);
    }

    /**
     * Comparison of another matrix with this one.
     *
     * \param  other  The other matrix that will be compared to this one
     * \return        Whether the two matrixs are unequal.
     */
    bool operator!=(const Matrix3x3<V> &other) const { return !(*this == other); }

    /**
     * Array-like member access.
     * \param  i index
     * \return reference to column
     */
    Vector3<V> &operator[](int i) { return c[i]; }

    /**
     * Transpose the matrix
     *
     * \return  A new object containing transposed matrix
     */
    Matrix3x3<V> transpose() const {
        return Matrix3x3<V>(Vector3<V>(c[0].x, c[1].x, c[2].x), Vector3<V>(c[0].y, c[1].y, c[2].y),
                            Vector3<V>(c[0].z, c[1].z, c[2].z));
    }

    /**
     * Calculation of the determinant of this matrix.
     *
     * \return The determinant.
     */
    V det() const {
        return c[0].x * (c[1].y * c[2].z - c[1].z * c[2].y) + c[0].y * (c[1].z * c[2].x - c[1].x * c[2].z) +
               c[0].z * (c[1].x * c[2].y - c[1].y * c[2].x);
    }

    /**
     * Calculate determinant of 2x2 Submatrix
     * | a b |
     * | c d |
     *
     * \return  determinant.
     */
    static V det2(V a, V b, V c, V d) { return a * d - b * c; }

    /**
     * Calculate the adjoint of this matrix.
     *
     * \return the adjoint matrix.
     */
    Matrix3x3<V> adjoint() const {
        return Matrix3x3<V>(Vector3<V>(det2(c[1].y, c[2].y, c[1].z, c[2].z), det2(c[2].x, c[1].x, c[2].z, c[1].z),
                                       det2(c[1].x, c[2].x, c[1].y, c[2].y)),
                            Vector3<V>(det2(c[2].y, c[0].y, c[2].z, c[0].z), det2(c[0].x, c[2].x, c[0].z, c[2].z),
                                       det2(c[2].x, c[0].x, c[2].y, c[0].y)),
                            Vector3<V>(det2(c[0].y, c[1].y, c[0].z, c[1].z), det2(c[1].x, c[0].x, c[1].z, c[0].z),
                                       det2(c[0].x, c[1].x, c[0].y, c[1].y)));
    }

    /**
     * Calculate the inverse of this matrix.
     *
     * \return The inverse matrix
     */
    Matrix3x3<V> invert() const { return adjoint().transpose() / det(); }
};

/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix : public Matrix3x3<double> {
   public:
    /**
     * Default constructor.
     */
    RotationMatrix() {}

    /**
     * Constructor.
     *
     * \param  c0  the first column of the matrix.
     * \param  c1  the second column of the matrix.
     * \param  c2  the third column of the matrix.
     */
    RotationMatrix(const Vector3<double> &c0, const Vector3<double> &c1, const Vector3<double> &c2)
        : Matrix3x3<double>(c0, c1, c2) {}

    /**
     * Assignment operator.
     *
     * \param  other  The other matrix that is assigned to this one
     * \return        A reference to this object after the assignment.
     */
    RotationMatrix &operator=(const Matrix3x3<double> &other) {
        c[0] = other.c[0];
        c[1] = other.c[1];
        c[2] = other.c[2];
        return *this;
    }

    /**
     * Copy constructor.
     *
     * \param  other  The other matrix that is copied to this one
     */
    RotationMatrix(const Matrix3x3<double> &other) { *this = other; }

    /**
     * RotationMatrix from RPY-angles.
     *   Roll  rotates along z axis,
     *   Pitch rotates along y axis,
     *   Yaw   rotates along x axis
     *
     *   R(roll,pitch,yaw) = R(z,roll)*R(y,pitch)*R(x,yaw)
     *
     * \see  "Robotik 1 Ausgabe Sommersemester 2001" by Prof. Dr. O. von Stryk
     * \attention  RPY-angles are not clearly defined!
     */
    RotationMatrix &fromKardanRPY(const double yaw, const double pitch, const double roll) {
        double cy = cos(yaw);
        double sy = sin(yaw);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double cr = cos(roll);
        double sr = sin(roll);

        c[0].x = cr * cp;
        c[0].y = -sr * cy + cr * sp * sy;
        c[0].z = sr * sy + cr * sp * cy;
        c[1].x = sr * cp;
        c[1].y = cr * cy + sr * sp * sy;
        c[1].z = -cr * sy + sr * sp * cy;
        c[2].x = -sp;
        c[2].y = cp * sy;
        c[2].z = cp * cy;

        return *this;
    }

    /**
     * Invert the matrix.
     *
     * \note: Inverted rotation matrix is transposed matrix.
     */
    RotationMatrix invert() { return transpose(); }

    /**
     * Rotation around the x-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix &rotateX(const double angle) {
        double c = cos(angle), s = sin(angle);
        *this *= RotationMatrix(Vector3<double>(1, 0, 0), Vector3<double>(0, c, s), Vector3<double>(0, -s, c));
        return *this;
    }

    /**
     * Rotation around the y-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix &rotateY(const double angle) {
        double c = cos(angle), s = sin(angle);
        *this *= RotationMatrix(Vector3<double>(c, 0, -s), Vector3<double>(0, 1, 0), Vector3<double>(s, 0, c));
        return *this;
    }

    /**
     * Rotation around the z-axis.
     *
     * \param   angle  The angle this pose will be rotated by
     * \return  A reference to this object after the calculation.
     */
    RotationMatrix &rotateZ(const double angle) {
        double c = cos(angle), s = sin(angle);
        *this *= RotationMatrix(Vector3<double>(c, s, 0), Vector3<double>(-s, c, 0), Vector3<double>(0, 0, 1));
        return *this;
    }

    /**
     * Get the x-angle of a RotationMatrix.
     *
     * \return  The angle around the x-axis between the original
     *          and the rotated z-axis projected on the y-z-plane
     */
    double getXAngle() const {
        double h = sqrt(c[2].y * c[2].y + c[2].z * c[2].z);
        return h ? acos(c[2].z / h) * (c[2].y > 0 ? -1 : 1) : 0;
    }

    /**
     * Get the y-angle of a RotationMatrix.
     *
     * \return  The angle around the y-axis between the original
     *          and the rotated x-axis projected on the x-z-plane
     */
    double getYAngle() const {
        double h = sqrt(c[0].x * c[0].x + c[0].z * c[0].z);
        return h ? acos(c[0].x / h) * (c[0].z > 0 ? -1 : 1) : 0;
    }

    /**
     * Get the z-angle of a RotationMatrix.
     *
     * \return  The angle around the z-axis between the original
     *          and the rotated x-axis projected on the x-y-plane
     */
    double getZAngle() const {
        double h = sqrt(c[0].x * c[0].x + c[0].y * c[0].y);
        return h ? acos(c[0].x / h) * (c[0].y < 0 ? -1 : 1) : 0;
    }

    /**
     * Create and return a RotationMatrix, rotated around x-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix getRotationX(const double angle) { return RotationMatrix().rotateX(angle); }

    /**
     * Create and return a RotationMatrix, rotated around y-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix getRotationY(const double angle) { return RotationMatrix().rotateY(angle); }

    /**
     * Create and return a RotationMatrix, rotated around z-axis
     *
     * \param   angle
     * \return  rotated RotationMatrix
     */
    static RotationMatrix getRotationZ(const double angle) { return RotationMatrix().rotateZ(angle); }
};

/** representation for 3D Transformation (Location + Orientation)*/
class Pose3D {
   public:
    /** rotation as a RotationMatrix*/
    RotationMatrix rotation;

    /** translation as a Vector3*/
    Vector3<double> translation;

    /** constructor*/
    Pose3D() {}

    /** constructor from rotation and translation
     * \param rot Rotation
     * \param trans Translation
     */
    Pose3D(const RotationMatrix &rot, const Vector3<double> &trans) : rotation(rot), translation(trans) {}

    /** constructor from rotation
     * \param rot Rotation
     */
    Pose3D(const RotationMatrix &rot) : rotation(rot) {}

    /** constructor from translation
     * \param trans Translation
     */
    Pose3D(const Vector3<double> &trans) : translation(trans) {}

    /** constructor from three translation values
     * \param x translation x component
     * \param y translation y component
     * \param z translation z component
     */
    Pose3D(const double x, const double y, const double z) : translation(x, y, z) {}

    /** Assignment operator
     *\param other The other Pose3D that is assigned to this one
     *\return A reference to this object after the assignment.
     */
    Pose3D &operator=(const Pose3D &other) {
        rotation    = other.rotation;
        translation = other.translation;

        return *this;
    }

    /** Copy constructor
     *\param other The other vector that is copied to this one
     */
    Pose3D(const Pose3D &other) { *this = other; }

    /** Multiplication with Point
     *\param point (Vector3&lt;double&gt;)
     */
    Vector3<double> operator*(const Vector3<double> &point) const { return rotation * point + translation; }

    /** Comparison of another vector with this one.
     *\param other The other vector that will be compared to this one
     *\return Whether the two vectors are equal.
     */
    bool operator==(const Pose3D &other) const {
        return ((translation == other.translation) && (rotation == other.rotation));
    }

    /** Comparison of another vector with this one.
     *\param other The other vector that will be compared to this one
     *\return Whether the two vectors are unequal.
     */
    bool operator!=(const Pose3D &other) const { return !(*this == other); }

    /**Concatenation of this pose with another pose
     *\param other The other pose that will be concatenated to this one.
     *\return A reference to this pose after concatenation
     */
    Pose3D &conc(const Pose3D &other) {
        translation = *this * other.translation;
        rotation *= other.rotation;
        return *this;
    }

    /** Calculates the inverse transformation from the current pose
     * @return The inverse transformation pose.
     */
    Pose3D invert() const {
        Pose3D result;
        result.rotation    = this->rotation.transpose();
        result.translation = Vector3<double>(0, 0, 0) - this->translation;
        return result;
    }

    /**Translate this pose by a translation vector
     *\param trans Vector to translate with
     *\return A reference to this pose after translation
     */
    Pose3D &translate(const Vector3<double> &trans) {
        translation = *this * trans;
        return *this;
    }

    /**Translate this pose by a translation vector
     *\param x x component of vector to translate with
     *\param y y component of vector to translate with
     *\param z z component of vector to translate with
     *\return A reference to this pose after translation
     */
    Pose3D &translate(const double x, const double y, const double z) {
        translation = *this * Vector3<double>(x, y, z);
        return *this;
    }

    /**Rotate this pose by a rotation
     *\param rot Rotationmatrix to rotate with
     *\return A reference to this pose after rotation
     */
    Pose3D &rotate(const RotationMatrix &rot) {
        rotation *= rot;
        return *this;
    }

    /**Rotate this pose around its x-axis
     *\param angle angle to rotate with
     *\return A reference to this pose after rotation
     */
    Pose3D &rotateX(const double angle) {
        rotation.rotateX(angle);
        return *this;
    }

    /**Rotate this pose around its y-axis
     *\param angle angle to rotate with
     *\return A reference to this pose after rotation
     */
    Pose3D &rotateY(const double angle) {
        rotation.rotateY(angle);
        return *this;
    }

    /**Rotate this pose around its z-axis
     *\param angle angle to rotate with
     *\return A reference to this pose after rotation
     */
    Pose3D &rotateZ(const double angle) {
        rotation.rotateZ(angle);
        return *this;
    }
};

typedef Vector2<double> Vector2D;
typedef Vector3<double> Vector3D;
typedef Range<double> RangeD;

}  // end of namespace xform

#define PEEK_CMD(cmd, s) (strcmp(cmd, s) == 0)
#define PEEK_CMD_N(cmd, s, n) (strncmp(cmd, s, n) == 0)
#define SCAN_CMD_D(cmd, d) (sscanf(cmd, "%d", &d))
#define SCAN_CMD_DD(cmd, d1, d2) (sscanf(cmd, "%d %d", &d1, &d2))
#define SCAN_CMD_DF(cmd, d, f) (sscanf(cmd, "%d %f", &d, &f))
#define SCAN_CMD_DDF(cmd, d1, d2, f) (sscanf(cmd, "%d %d %f", &d1, &d2, &f))
#define PEEK_CMD_FF(cmd, s, n, f1, f2) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f", &f1, &f2) == 2)
#define PEEK_CMD_FFF(cmd, s, n, f1, f2, f3) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f", &f1, &f2, &f3) == 3)
#define PEEK_CMD_D(cmd, s, n, d) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d", &d) == 1)
#define PEEK_CMD_DD(cmd, s, n, d1, d2) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %d", &d1, &d2) == 2)
#define PEEK_CMD_DDD(cmd, s, n, d1, d2, d3) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %d %d", &d1, &d2, &d3) == 3)
#define PEEK_CMD_DDDD(cmd, s, n, d1, d2, d3, d4) \
    (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %d %d %d", &d1, &d2, &d3, &d4) == 4)
#define PEEK_CMD_FFFF(cmd, s, n, d1, d2, d3, d4) \
    (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f %f", &d1, &d2, &d3, &d4) == 4)
#define PEEK_CMD_FFFFF(cmd, s, n, d1, d2, d3, d4, d5) \
    (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f %f %f", &d1, &d2, &d3, &d4, &d5) == 5)
#define PEEK_CMD_FFFFD(cmd, s, n, d1, d2, d3, d4, d5) \
    (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %f %f %f %d", &d1, &d2, &d3, &d4, &d5) == 5)
#define PEEK_CMD_S(cmd, s, n, str) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%s", str) == 1)
#define PEEK_CMD_SS(cmd, s, n, str1, str2) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%s %s", str1, str2) == 2)
#define PEEK_CMD_SD(cmd, s, n, str1, d) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%s %d", str1, &d) == 2)
#define PEEK_CMD_DS(cmd, s, n, d, str) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%d %s", &d, str) == 2)
#define PEEK_CMD_FS(cmd, s, n, f, str) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f %s", &f, str) == 2)
#define PEEK_CMD_F(cmd, s, n, f) (strncmp(cmd, s, n) == 0 && sscanf(cmd + n, "%f", &f) == 1)

#endif
