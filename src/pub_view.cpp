#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <errno.h>
#include <float.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <visualization_msgs/msg/marker.hpp>

#define __VERBOSE 0

#define N_SAMPLES 128

#define N_FUNCTIONS 3

#include <errno.h>
#include <float.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define handle_error_en( en, msg )                                                                                     \
    do {                                                                                                               \
        errno = en;                                                                                                    \
        perror( msg );                                                                                                 \
        exit( EXIT_FAILURE );                                                                                          \
    } while( 0 )

#define MAX_LINE_LEN                                                                                                   \
    ( 3 - DBL_MIN_10_EXP + 1 ) * 3 + 10  // Tamanho maximo que uma linha pode ter. Assumindo 3xDLOUBLE + 10.

//(strlen("-0.")-DBL_MIN_10_EXP+1) Sendo o maior valor que uma variavel double pode assumir em ascii

typedef struct
{
    double *x, *y, *z;
    int npoints;
} t_point_cloud;

void read_point_cloud( t_point_cloud** ptr, char* file_name )
{
    ( *ptr )          = (t_point_cloud*) malloc( sizeof( t_point_cloud ) );
    ( *ptr )->npoints = 0;
    ( *ptr )->x       = NULL;
    ( *ptr )->y       = NULL;
    ( *ptr )->z       = NULL;

    FILE* file        = NULL;
    if( file == NULL ) file = fopen( file_name, "r" );  // Abre o ficheiro na primeira vez que é chamada
    else perror( "Missing input file." );

#if __VERBOSE == 1
    printf( "File name: %s\n", file_name );
#endif
    int count = 0;
    char line_buffer[ MAX_LINE_LEN ];
    while( fgets( line_buffer, sizeof( line_buffer ), file ) != NULL ) count++;
    fseek( file, 0, SEEK_SET );
    ( *ptr )->npoints = count;
    ( *ptr )->x       = (double*) malloc( sizeof( double ) * ( *ptr )->npoints );
    ( *ptr )->y       = (double*) malloc( sizeof( double ) * ( *ptr )->npoints );
    ( *ptr )->z       = (double*) malloc( sizeof( double ) * ( *ptr )->npoints );
    char* ptr_c;
    count = 0;
    while( fgets( line_buffer, sizeof( line_buffer ), file ) != NULL )
    {
#if __VERBOSE == 1
        printf( "Buff: %s\n", line_buffer );
#endif
        ptr_c                = strtok( line_buffer, " " );
        ( *ptr )->x[ count ] = atof( ptr_c );
        ptr_c                = strtok( NULL, " " );
        ( *ptr )->y[ count ] = atof( ptr_c );
        ptr_c                = strtok( NULL, " " );
        ( *ptr )->z[ count ] = atof( ptr_c );
        count++;
    }

#if __VERBOSE == 1
    printf( "Count: %i\n", count );
#endif
    fclose( file );
    // return ret_ptr;
}

void describe_point_cloud( t_point_cloud* ptr )
{
    double max[ 3 ] = { ptr->x[ 0 ], ptr->y[ 0 ], ptr->z[ 0 ] }, min[ 3 ] = { ptr->x[ 0 ], ptr->y[ 0 ], ptr->z[ 0 ] },
                  mean[ 3 ] = { 0, 0, 0 }, std[ 3 ] = { 0, 0, 0 };

    for( int i = 0; i < ptr->npoints; i++ )
    {
        // Max
        if( ptr->x[ i ] > max[ 0 ] ) max[ 0 ] = ptr->x[ i ];
        if( ptr->y[ i ] > max[ 1 ] ) max[ 1 ] = ptr->y[ i ];
        if( ptr->z[ i ] > max[ 2 ] ) max[ 2 ] = ptr->z[ i ];
        // Min
        if( ptr->x[ i ] < min[ 0 ] ) min[ 0 ] = ptr->x[ i ];
        if( ptr->y[ i ] < min[ 1 ] ) min[ 1 ] = ptr->y[ i ];
        if( ptr->z[ i ] < min[ 2 ] ) min[ 2 ] = ptr->z[ i ];
        // Mean
        mean[ 0 ] += ptr->x[ i ];
        mean[ 1 ] += ptr->y[ i ];
        mean[ 2 ] += ptr->z[ i ];
    }
    mean[ 0 ] /= ptr->npoints;
    mean[ 1 ] /= ptr->npoints;
    mean[ 2 ] /= ptr->npoints;

    for( int i = 0; i < ptr->npoints; i++ )
    {
        std[ 0 ] = pow( ptr->x[ i ] - mean[ 0 ], 2 );
        std[ 1 ] = pow( ptr->y[ i ] - mean[ 1 ], 2 );
        std[ 2 ] = pow( ptr->z[ i ] - mean[ 2 ], 2 );
    }
    std[ 0 ] /= ptr->npoints;
    std[ 1 ] /= ptr->npoints;
    std[ 2 ] /= ptr->npoints;
    std[ 0 ]  = sqrt( std[ 0 ] );
    std[ 1 ]  = sqrt( std[ 1 ] );
    std[ 2 ]  = sqrt( std[ 2 ] );

    printf( "\n\nReport:" );
    printf( "\tCount: %i\n", ptr->npoints );
    printf( "\t      [         x,         y,         z]\n" );
    printf( "\tMax:  [%10.6f,%10.6f,%10.6f]\n", max[ 0 ], max[ 1 ], max[ 2 ] );
    printf( "\tMin:  [%10.6f,%10.6f,%10.6f]\n", min[ 0 ], min[ 1 ], min[ 2 ] );
    printf( "\tMean: [%10.6f,%10.6f,%10.6f]\n", mean[ 0 ], mean[ 1 ], mean[ 2 ] );
    printf( "\tSTD:  [%10.6f,%10.6f,%10.6f]\n", std[ 0 ], std[ 1 ], std[ 2 ] );
}

void free_t_point_cloud( t_point_cloud* ptr )
{
    free( ptr->x );
    free( ptr->y );
    free( ptr->z );
    free( ptr );
}

void filter_point_cloud( t_point_cloud** ptr )
{
    const int n_rules = 4;
    int i, count = 0;
    int* valid_pts = (int*) malloc( sizeof( int ) * ( *ptr )->npoints );

    // looking for valid poiunts
    for( i = 0; i < ( *ptr )->npoints; i++ )
    {
        valid_pts[ i ] = n_rules;

        // (a) remove x < 0 points
        if( ( *ptr )->x[ i ] >= 0 ) valid_pts[ i ]--;

        // (b) remove r <= 1.6 points
        if( sqrt( pow( ( *ptr )->x[ i ], 2 ) + pow( ( *ptr )->y[ i ], 2 ) + pow( ( *ptr )->z[ i ], 2 ) ) > 1.6 )
            valid_pts[ i ]--;

        // (c) remove z > -0.3
        if( ( *ptr )->z[ i ] <= -0.3 ) valid_pts[ i ]--;

        // filter 110 deg in front of the car
        // This rule already removes X < 0 points
        double ang = atan2( ( *ptr )->y[ i ], ( *ptr )->x[ i ] ) * 180 / M_PI;
        if( abs( ang ) <= 55 ) valid_pts[ i ]--;

        if( !valid_pts[ i ] ) count++;
    }

    // redefining the point_cloud
    int aux_count = 0;
    double* new_x = (double*) malloc( sizeof( double ) * count );
    double* new_y = (double*) malloc( sizeof( double ) * count );
    double* new_z = (double*) malloc( sizeof( double ) * count );

    for( i = 0; i < ( *ptr )->npoints; i++ )
    {
        if( !valid_pts[ i ] )
        {
            new_x[ aux_count ] = ( *ptr )->x[ i ];
            new_y[ aux_count ] = ( *ptr )->y[ i ];
            new_z[ aux_count ] = ( *ptr )->z[ i ];
            aux_count++;
        }
    }
#if __VERBOSE == 1
    printf( "Old Size: %i\n", aux_count );
#endif
    free( ( *ptr )->x );
    free( ( *ptr )->y );
    free( ( *ptr )->z );
    ( *ptr )->x       = new_x;
    ( *ptr )->y       = new_y;
    ( *ptr )->z       = new_z;
    ( *ptr )->npoints = aux_count;

#if __VERBOSE == 1
    printf( "New Size: %i\n", count );
#endif

    free( valid_pts );
}

void filter_roads( t_point_cloud** ptr, const int n_bins )
{

    int i;
    double lims[ 2 ]  = { DBL_MAX, DBL_MIN };
    const int npoints = ( *ptr )->npoints;
    // const int npoints = 20;
    for( i = 0; i < npoints; i++ )
    {
        if( ( *ptr )->z[ i ] > lims[ 1 ] ) lims[ 1 ] = ( *ptr )->z[ i ];
        if( ( *ptr )->z[ i ] < lims[ 0 ] ) lims[ 0 ] = ( *ptr )->z[ i ];
    }

    // printf("Min: %f, Max: %f\n",lims[0],lims[1]);
    double gran = ( lims[ 1 ] - lims[ 0 ] ) / n_bins;
    int* bins   = NULL;
    bins        = (int*) malloc( sizeof( int ) * n_bins );
    for( i = 0; i < n_bins; i++ ) bins[ i ] = 0;

    int c_bin      = 0;
    int* bin_numer = (int*) malloc( sizeof( int ) * ( *ptr )->npoints );
    for( i = 0; i < npoints; i++ )
    {
        c_bin = (int) floor(
            ( ( *ptr )->z[ i ] >= 0 ? ( *ptr )->z[ i ] + lims[ 0 ] : ( *ptr )->z[ i ] - lims[ 0 ] ) / gran );
        bins[ c_bin ]++;
        bin_numer[ i ] = c_bin;
    }

    // printf("Bins:\n");
    int max_bin = 0;
    for( i = 0; i < n_bins; i++ )
    {
        // printf("|%i",bins[i]);
        if( bins[ max_bin ] < bins[ i ] ) max_bin = i;
    }

    int selected_bins[ 3 ] = { -1, -1, -1 };
    // mask [c3,c2,c1,cm3,cm2,cm1]
    u_int8_t mask = 0b010;  // binary mask
    if( max_bin > 0 )
    {
        selected_bins[ 0 ]  = max_bin - 1;
        mask               |= 0b001;
    }
    selected_bins[ 1 ] = max_bin;
    if( max_bin + 1 <= n_bins )
    {
        selected_bins[ 2 ]  = max_bin + 1;
        mask               |= 0b100;
    }

#if __VERBOSE == 1
    printf( "\nSelected bins: [%i,%i,%i]", selected_bins[ 0 ], selected_bins[ 1 ], selected_bins[ 2 ] );

    printf(
        "\nLims bins: [%f,%f], [%f,%f], [%f,%f]\n", lims[ 1 ] + selected_bins[ 0 ] * gran,
        lims[ 1 ] + ( selected_bins[ 0 ] + 1 ) * gran, lims[ 1 ] + selected_bins[ 1 ] * gran,
        lims[ 1 ] + ( selected_bins[ 1 ] + 1 ) * gran, lims[ 1 ] + selected_bins[ 2 ] * gran,
        lims[ 1 ] + ( selected_bins[ 2 ] + 1 ) * gran );

    printf( "\nMax bin %i\n", max_bin );
#endif

    int count = 0;  // Expected number of points within the center bins
    count     = ( ( ( 0b001 & mask ) == 0b001 ) ? bins[ selected_bins[ 0 ] ] : 0 )
          + ( ( ( 0b010 & mask ) == 0b010 ) ? bins[ selected_bins[ 1 ] ] : 0 )
          + ( ( ( 0b100 & mask ) == 0b100 ) ? bins[ selected_bins[ 2 ] ] : 0 );
    int aux_count = 0;
    double* new_x = (double*) malloc( sizeof( double ) * count );
    double* new_y = (double*) malloc( sizeof( double ) * count );
    double* new_z = (double*) malloc( sizeof( double ) * count );

    for( i = 0; i < ( *ptr )->npoints; i++ )
    {

        for( int j = 0; j < 3; j++ )
        {
            if( j == 0 && ( mask & 0b001 ) == 0b000 ) continue;
            if( j == 1 && ( mask & 0b010 ) == 0b000 ) continue;
            if( j == 2 && ( mask & 0b100 ) == 0b000 ) continue;

            if( selected_bins[ j ] == bin_numer[ i ] )
            {
                new_x[ aux_count ] = ( *ptr )->x[ i ];
                new_y[ aux_count ] = ( *ptr )->y[ i ];
                new_z[ aux_count ] = ( *ptr )->z[ i ];
                aux_count++;
                break;
            }
        }
    }

    free( ( *ptr )->x );
    free( ( *ptr )->y );
    free( ( *ptr )->z );
    ( *ptr )->x       = new_x;
    ( *ptr )->y       = new_y;
    ( *ptr )->z       = new_z;
    ( *ptr )->npoints = count;

    free( bins );
    free( bin_numer );

    // double bins[]
}

using namespace std::placeholders;

class pub_view : public rclcpp::Node
{
  public:

    pub_view() : Node( "demo_publisher" ), count_( 0 )
    {
        publisher_1               = this->create_publisher< sensor_msgs::msg::PointCloud2 >( "Pcloud", 10 );
        publisher_2               = this->create_publisher< sensor_msgs::msg::PointCloud2 >( "Pcloud_filt", 10 );
        publisher_3               = this->create_publisher< sensor_msgs::msg::PointCloud2 >( "Pcloud_road_filt", 10 );

        marker_pub                = this->create_publisher< visualization_msgs::msg::Marker >( "Origin", 10 );

        marker.header.frame_id    = "map";
        marker.header.stamp       = this->get_clock().get()->now();
        marker.id                 = 0;
        marker.type               = visualization_msgs::msg::Marker::SPHERE;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = 0;
        marker.pose.position.y    = 0;
        marker.pose.position.z    = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0;
        marker.scale.x            = 2.5;
        marker.scale.y            = 2.5;
        marker.scale.z            = 2.5;
        marker.color.r            = 1.0;
        marker.color.g            = 0.0;
        marker.color.b            = 0.0;
        marker.color.a            = 0.5;

        filter_point_cloud( &this->pointCloud2 );
        filter_point_cloud( &this->pointCloud3 );
        filter_roads( &this->pointCloud3, 12 );
        describe_point_cloud( this->pointCloud2 );
        describe_point_cloud( this->pointCloud3 );

        ptc_msg( this->msg1, this->pointCloud1, 0, 100, 0 );
        ptc_msg( this->msg2, this->pointCloud2, 255, 0, 200 );
        ptc_msg( this->msg3, this->pointCloud3, 0, 0, 200 );

        publisher_1->publish( this->msg1 );
        publisher_2->publish( this->msg2 );
        publisher_3->publish( this->msg3 );
        marker_pub->publish( marker );

        timer_ =
            this->create_wall_timer( std::chrono::milliseconds( 2000 ), std::bind( &pub_view::timer_callback, this ) );
    }

    ~pub_view() { free_t_point_cloud( this->pointCloud1 ); }

  private:

    void timer_callback()
    {

        marker.action = visualization_msgs::msg::Marker::MODIFY;
        marker_pub->publish( marker );
        this->msg1.header.stamp = this->get_clock().get()->now();
        publisher_1->publish( this->msg1 );
        this->msg2.header.stamp = this->get_clock().get()->now();
        publisher_2->publish( this->msg2 );
        this->msg3.header.stamp = this->get_clock().get()->now();
        publisher_3->publish( this->msg3 );
    }

    void ptc_msg( sensor_msgs::msg::PointCloud2& msg, t_point_cloud* data, uint8_t r, uint8_t g, uint8_t b )
    {
        msg.header.frame_id = "map";
        msg.header.stamp    = this->get_clock().get()->now();
        msg.height          = 1;
        msg.width           = data->npoints;
        msg.point_step      = ( sizeof( float ) * 3 );
        msg.is_dense        = true;
        msg.row_step        = msg.width;
        msg.is_bigendian    = false;
        msg.data.resize( msg.height * msg.width );

        sensor_msgs::PointCloud2Modifier mod( msg );

        mod.setPointCloud2FieldsByString( 2, "xyz", "rgb" );
        mod.resize( msg.height * msg.width );

        sensor_msgs::PointCloud2Iterator< float > iter_x( msg, "x" );
        sensor_msgs::PointCloud2Iterator< float > iter_y( msg, "y" );
        sensor_msgs::PointCloud2Iterator< float > iter_z( msg, "z" );

        sensor_msgs::PointCloud2Iterator< uint8_t > iter_r( msg, "r" );
        sensor_msgs::PointCloud2Iterator< uint8_t > iter_g( msg, "g" );
        sensor_msgs::PointCloud2Iterator< uint8_t > iter_b( msg, "b" );

        int i = 0;
        while( iter_x != iter_x.end() )
        {
            *iter_x = (float) data->x[ i ];
            *iter_y = (float) data->y[ i ];
            *iter_z = (float) data->z[ i ];

            *iter_r = (uint8_t) r;
            *iter_g = (uint8_t) g;
            *iter_b = (uint8_t) b;

            ++iter_x;
            ++iter_y;
            ++iter_z;
            i++;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr publisher_1;
    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr publisher_2;
    rclcpp::Publisher< sensor_msgs::msg::PointCloud2 >::SharedPtr publisher_3;
    size_t count_;
    t_point_cloud* pointCloud1;
    t_point_cloud* pointCloud2;
    t_point_cloud* pointCloud3;
    sensor_msgs::msg::PointCloud2 msg1;
    sensor_msgs::msg::PointCloud2 msg2;
    sensor_msgs::msg::PointCloud2 msg3;
    visualization_msgs::msg::Marker marker;
    rclcpp::Publisher< visualization_msgs::msg::Marker >::SharedPtr marker_pub;
};

int main( int argc, char* argv[] )
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared< pub_view >() );
    rclcpp::shutdown();
    return 0;
}
