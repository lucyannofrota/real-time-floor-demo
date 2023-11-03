#include "../include/floor_demo/aux_functions.hpp"

void read_point_cloud( t_point_cloud** ptr, char* file_name )
{
    ( *ptr )->npoints = 0;
    free( ( *ptr )->x );
    free( ( *ptr )->y );
    free( ( *ptr )->z );

    FILE* file = fopen( file_name, "r" );
    if( file == NULL ) handle_error_en( errno, "fopen()" );

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
        if( fabs( ang ) <= 55.0 ) valid_pts[ i ]--;

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
    for( i = 0; i < npoints; i++ )
    {
        if( ( *ptr )->z[ i ] > lims[ 1 ] ) lims[ 1 ] = ( *ptr )->z[ i ];
        if( ( *ptr )->z[ i ] < lims[ 0 ] ) lims[ 0 ] = ( *ptr )->z[ i ];
    }

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

    int max_bin = 0;
    for( i = 0; i < n_bins; i++ )
        if( bins[ max_bin ] < bins[ i ] ) max_bin = i;

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
}

void clk_wait( double m_sec )
{
    // https://stackoverflow.com/questions/20332382/linux-sleeping-with-clock-nanosleep

    struct timespec deadline;
    clock_gettime( CLOCK_REALTIME, &deadline );

    // Add the time you want to sleep
    deadline.tv_nsec += (long) ceil( m_sec * 1000000000 / 1000.0 );

    // Normalize the time to account for the second boundary
    if( deadline.tv_nsec >= 1000000000 )
    {
        deadline.tv_nsec -= 1000000000;
        deadline.tv_sec++;
    }
    clock_nanosleep( CLOCK_REALTIME, TIMER_ABSTIME, &deadline, NULL );
}

void add_timespec( const struct timespec* tim_1, const struct timespec* tim_2, struct timespec* result )
{
    // from <sys/time.h>
    // define timeradd(a, b, result)
    result->tv_sec  = tim_1->tv_sec + tim_2->tv_sec;
    result->tv_nsec = tim_1->tv_nsec + tim_2->tv_nsec;
    if( result->tv_nsec >= 1e9 )
    {
        result->tv_sec++;
        result->tv_nsec -= 1e9;
    }
}

void sub_timespec( struct timespec* tim_1, struct timespec* tim_2, struct timespec* result )
{
    result->tv_sec  = tim_1->tv_sec - tim_2->tv_sec;
    result->tv_nsec = tim_1->tv_nsec - tim_2->tv_nsec;
    if( result->tv_nsec < 0 )
    {
        result->tv_sec  -= 1;
        result->tv_nsec += 1000000000;
    }
}

double dtime_ms( const struct timespec* tim_1, const struct timespec* tim_2 )
{
    // from <sys/time.h>
    // # define timersub(a, b, result)
    struct timespec result;
    result.tv_sec  = tim_1->tv_sec - tim_2->tv_sec;
    result.tv_nsec = tim_1->tv_nsec - tim_2->tv_nsec;
    if( result.tv_nsec < 0 && result.tv_sec > 0 )
    {
        result.tv_sec--;
        result.tv_nsec += 1e9;
    }
    return ( result.tv_sec * 1e9 + ( (long int) result.tv_nsec ) ) / 1000000.0;
}

void print_timespec( struct timespec t, char* prefix )
{
    printf( "%s sec: %i\n%snsec: %ld\n", prefix, (int) t.tv_sec, prefix, t.tv_nsec );
}

void display_thread_attr( pthread_t thread, char* prefix )
{
    // https://man7.org/linux/man-pages/man3/pthread_getattr_np.3.html
    int s, i;
    pthread_attr_t attr;
    // pthread_attr_init(&attr);

    s = pthread_getattr_np( thread, &attr );
    if( s != 0 ) handle_error_en( s, "pthread_getattr_np" );

    // https://man7.org/linux/man-pages/man3/pthread_getattr_np.3.html

    // https://man7.org/linux/man-pages/man3/pthread_attr_init.3.html
    // https://man7.org/linux/man-pages/man3/pthread_attr_destroy.3.html
    struct sched_param sp;

    cpu_set_t cpu_set;

    s = pthread_attr_getaffinity_np( &attr, sizeof( cpu_set ), &cpu_set );
    if( s != 0 ) handle_error_en( s, "pthread_attr_getaffinity_np" );
    printf( "%sThread affinity = %d\n", prefix, CPU_COUNT( &cpu_set ) );

    s = pthread_attr_getinheritsched( &attr, &i );
    if( s != 0 ) handle_error_en( s, "pthread_attr_getinheritsched" );

    printf(
        "%sInherit scheduler   = %s\n", prefix,
        ( i == PTHREAD_INHERIT_SCHED )    ? "PTHREAD_INHERIT_SCHED"
        : ( i == PTHREAD_EXPLICIT_SCHED ) ? "PTHREAD_EXPLICIT_SCHED"
                                          : "???" );

    s = pthread_attr_getschedpolicy( &attr, &i );
    if( s != 0 ) handle_error_en( s, "pthread_attr_getschedpolicy" );
    printf(
        "%sScheduling policy   = %s\n", prefix,
        ( i == SCHED_OTHER )  ? "SCHED_OTHER"
        : ( i == SCHED_FIFO ) ? "SCHED_FIFO"
        : ( i == SCHED_RR )   ? "SCHED_RR"
                              : "???" );

    s = pthread_attr_getschedparam( &attr, &sp );
    if( s != 0 ) handle_error_en( s, "pthread_attr_getschedparam" );
    printf( "%sScheduling priority = %d\n", prefix, sp.sched_priority );

    // https://man7.org/linux/man-pages/man3/pthread_attr_init.3.html
    // https://man7.org/linux/man-pages/man3/pthread_attr_destroy.3.html

    s = pthread_attr_destroy( &attr );
    if( s != 0 ) handle_error_en( s, "pthread_attr_destroy" );
}

double timespec_to_double_ms( struct timespec* time )
{
    return ( ( time->tv_sec * 1000000000 + time->tv_nsec ) / 1000000000.0 ) * 1000;  // ms
}

void print_table( struct timespec* tab, int M, int N, char* prefix )
{
    double* max_values  = (double*) malloc( sizeof( double ) * M );
    double* mean_values = (double*) malloc( sizeof( double ) * M );

    // Max values initialization
    int i, j;

    for( i = 0; i < M; i++ )
    {
        max_values[ i ]  = 0;
        mean_values[ i ] = 0;
    }

    // Print time table
    printf( "%sTime table (%i samples) [ms]:\n", prefix, N );
    for( i = 0; i < N; i++ )
    {
        printf( "%s\t", prefix );
        for( j = 0; j < M; j++ )
        {
            mean_values[ j ] += timespec_to_double_ms( ( tab + ( j * N + i ) ) );
            if( max_values[ j ] < timespec_to_double_ms( ( tab + ( j * N + i ) ) ) )
                max_values[ j ] = timespec_to_double_ms( ( tab + ( j * N + i ) ) );
            printf( "| f%i = %9.3f ", j + 1, timespec_to_double_ms( ( tab + ( j * N + i ) ) ) );
        }
        printf( "|\n" );
    }
    // Print max values
    printf( "%sMax Times (%i samples) [ms]:\n", prefix, N );
    printf( "%s\t", prefix );
    int sum_max = 0;
    for( j = 0; j < M; j++ )
    {
        printf( "| f%i = %9.3f ", j + 1, max_values[ j ] );
        sum_max += max_values[ j ];
    }
    printf( "|\n" );
    printf( "%sMean Times (%i samples) [ms]:\n", prefix, N );
    printf( "%s\t", prefix );
    for( j = 0; j < M; j++ ) printf( "| f%i = %9.3f ", j + 1, mean_values[ j ] / N );
    printf( "|\n" );
    printf( "%sSum Max Times (%i functions) [ms]: %i\n", prefix, N_FUNCTIONS, sum_max );
    printf( "\n\n\n" );

    free( max_values );
    free( mean_values );
}

void thread_configs( pthread_attr_t* attr, int setaffinity, int sched_type, int priority_mod )
{
    // https://man7.org/linux/man-pages/man7/sched.7.html

    pthread_attr_init( attr );

    // Affinity

    if( setaffinity == 1 )
    {
        cpu_set_t cpu_set;

        CPU_ZERO( &cpu_set );
        CPU_SET( 0, &cpu_set );

        pthread_attr_setaffinity_np( attr, sizeof( cpu_set_t ), &cpu_set );
    }
    // Inheritance

    pthread_attr_setinheritsched( attr, PTHREAD_EXPLICIT_SCHED );

    /* Policy options:
    //  SCHED_FIFO
    //  SCHED_RR
    //  SCHED_OTHER
    */

    pthread_attr_setschedpolicy( attr, sched_type );

    struct sched_param param;

    param.sched_priority = sched_get_priority_max( sched_type ) + priority_mod;

    pthread_attr_setschedparam( attr, &param );
}

// Sem

void read_point_cloud_sem( t_point_cloud** ptr, char* file_name, sem_t* sem_b, sem_t* sem_a )
{
    sem_wait( sem_b );
    if( *ptr == NULL ) handle_error_en( errno, "ptr == NULL" );
    ( *ptr )->npoints = 0;
    free( ( *ptr )->x );
    free( ( *ptr )->y );
    free( ( *ptr )->z );

    FILE* file = fopen( file_name, "r" );
    if( file == NULL ) handle_error_en( errno, "fopen()" );

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
    sem_post( sem_a );

#if __VERBOSE == 1
    printf( "Count: %i\n", count );
#endif
    fclose( file );
}

void filter_point_cloud_sem( t_point_cloud** ptr, sem_t* sem_b, sem_t* sem_a )
{
    const int n_rules = 4;
    int i, count = 0;
    sem_wait( sem_b );
    int* valid_pts = (int*) malloc( sizeof( int ) * ( *ptr )->npoints );

    // looking for valid points
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
        if( fabs( ang ) <= 55.0 ) valid_pts[ i ]--;

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

    sem_post( sem_a );

#if __VERBOSE == 1
    printf( "New Size: %i\n", count );
#endif

    free( valid_pts );
}

void filter_roads_sem( t_point_cloud** ptr, const int n_bins, sem_t* sem_b, sem_t* sem_a )
{

    int i;
    double lims[ 2 ] = { DBL_MAX, DBL_MIN };
    sem_wait( sem_b );
    const int npoints = ( *ptr )->npoints;
    for( i = 0; i < npoints; i++ )
    {
        if( ( *ptr )->z[ i ] > lims[ 1 ] ) lims[ 1 ] = ( *ptr )->z[ i ];
        if( ( *ptr )->z[ i ] < lims[ 0 ] ) lims[ 0 ] = ( *ptr )->z[ i ];
    }

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

    int max_bin = 0;
    for( i = 0; i < n_bins; i++ )
        if( bins[ max_bin ] < bins[ i ] ) max_bin = i;

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

    sem_post( sem_a );

    free( bins );
    free( bin_numer );
}
