#ifndef TCP_LEDBAT_H
#define TCP_LEDBAT_H

#include "tcp-congestion-ops.h"

#include <vector>

namespace ns3
{

    class TcpSocketState;

    /**
     * @ingroup congestionOps
     *
     * @brief An implementation of LEDBAT++ with adaptive target delay and
     *        dynamic slowdown scheduling.
     *
     * Modifications over base LEDBAT++:
     *
     *  1. ADAPTIVE TARGET DELAY (addresses RFC §3.4 – Low Latency Competition)
     *     effective_target = clamp(2 * base_delay, MinTargetDelay, TargetDelay)
     *     On low-latency links the target shrinks so the connection still yields
     *     to foreground traffic. On WAN paths the target stays at 60 ms as before.
     *     ComputeEffectiveTarget() is used everywhere m_target was previously read.
     *
     *  2. DYNAMIC SLOWDOWN INTERVAL (improves RFC §4.4 – Periodic Slowdowns)
     *     Instead of always scheduling the next slowdown at 9x the measured
     *     slowdown duration, the multiplier is chosen based on queue pressure at
     *     slowdown exit:
     *       pressure = queueDelay / effectiveTarget  (clamped 0..1)
     *       multiplier = lerp(SlowdownMultMax, SlowdownMultMin, pressure)
     *     High pressure → multiplier → SlowdownMultMin (default 6, more frequent)
     *     Low  pressure → multiplier → SlowdownMultMax (default 12, less frequent)
     *     Setting both attributes to 9 restores the original RFC behaviour.
     */
    class TcpLedbatPlusPlusModified : public TcpNewReno
    {
    private:
        /**
         * @brief The slowstart types
         */
        enum SlowStartType
        {
            DO_NOT_SLOWSTART, //!< Do not Slow Start
            DO_SLOWSTART,     //!< Do NewReno Slow Start
        };

        /**
         * @brief The state of LEDBAT. If LEDBAT is not in VALID_OWD state, it falls to
         *        default congestion ops.
         */
        enum State : uint32_t
        {
            LEDBAT_VALID_OWD = (1 << 1), //!< If valid timestamps are present
            LEDBAT_CAN_SS = (1 << 3)     //!< If LEDBAT allows Slow Start
        };

    public:
        /**
         * @brief Get the type ID.
         * @return the object TypeId
         */
        static TypeId GetTypeId();

        /**
         * Create an unbound tcp socket.
         */
        TcpLedbatPlusPlusModified();

        /**
         * @brief Copy constructor
         * @param sock the object to copy
         */
        TcpLedbatPlusPlusModified(const TcpLedbatPlusPlusModified &sock);

        /**
         * @brief Destructor
         */
        ~TcpLedbatPlusPlusModified() override;

        /**
         * @brief Get the name of the TCP flavour
         * @return The name of the TCP
         */
        std::string GetName() const override;

        /**
         * @brief Get information from the acked packet
         * @param tcb internal congestion state
         * @param segmentsAcked count of segments ACKed
         * @param rtt The estimated rtt
         */
        void PktsAcked(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked, const Time &rtt) override;

        // Inherited
        Ptr<TcpCongestionOps> Fork() override;

        /**
         * @brief Adjust cwnd following LEDBAT++ algorithm
         * @param tcb internal congestion state
         * @param segmentsAcked count of segments ACKed
         */
        void IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) override;

        /**
         * @brief Change the Slow Start Capability
         * @param doSS Slow Start Option
         */
        void SetDoSs(SlowStartType doSS);

    protected:
        /**
         * @brief Reduce Congestion
         * @param tcb internal congestion state
         * @param segmentsAcked count of segments ACKed
         */
        void CongestionAvoidance(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) override;

        /**
         * @brief Exponential slow start using gain factor
         * @param tcb internal congestion state
         * @param segmentsAcked count of segments ACKed
         */
        uint32_t SlowStart(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked) override;

    private:
        /**
         * @brief Buffer structure to store delays
         */
        struct OwdCircBuf
        {
            std::vector<uint32_t> buffer; //!< Vector to store the delay
            size_t min;                   //!< The index of minimum value
        };

        /**
         * @brief Initialise a new buffer
         * @param buffer The buffer to be initialised
         */
        void InitCircBuf(OwdCircBuf &buffer);

        /// Filter function used by LEDBAT for current delay
        typedef uint32_t (*FilterFunction)(OwdCircBuf &);

        /**
         * @brief Return the minimum delay of the buffer
         * @param b The buffer
         * @return The minimum delay
         */
        static uint32_t MinCircBuf(OwdCircBuf &b);

        /**
         * @brief Return the value of current delay
         * @param filter The filter function
         * @return The current delay
         */
        uint32_t CurrentDelay(FilterFunction filter);

        /**
         * @brief Return the value of base delay
         * @return The base delay
         */
        uint32_t BaseDelay();

        /**
         * @brief Compute the dynamic GAIN parameter
         * @return GAIN value
         */
        double ComputeGain();

        /**
         * @brief Add new delay to the buffers
         * @param cb The buffer
         * @param owd The new delay
         * @param maxlen The maximum permitted length
         */
        void AddDelay(OwdCircBuf &cb, uint32_t owd, uint32_t maxlen);

        /**
         * @brief Update the base delay buffer
         * @param owd The delay
         */
        void UpdateBaseDelay(uint32_t owd);

        // -----------------------------------------------------------------------
        // MOD 1: Adaptive Target Delay
        // -----------------------------------------------------------------------

        /**
         * @brief Compute effective target delay for the current RTT sample.
         *
         * Returns clamp(2 * base_delay, m_minTarget, m_target).
         * Falls back to m_target when no base delay sample is available yet,
         * preserving original behaviour at connection start.
         *
         * @return Effective target delay as a Time value
         */
        Time ComputeEffectiveTarget() const;

        // -----------------------------------------------------------------------
        // MOD 2: Dynamic Slowdown Interval
        // -----------------------------------------------------------------------

        /**
         * @brief Compute the next-slowdown multiplier based on queue pressure.
         *
         * multiplier = lerp(m_slowdownMultMax, m_slowdownMultMin,
         *                   clamp(queueDelayAtExit / effectiveTarget, 0, 1))
         * High pressure → m_slowdownMultMin (more frequent slowdowns)
         * Low  pressure → m_slowdownMultMax (less frequent slowdowns)
         *
         * @param queueDelayAtExit Queue delay observed when slowdown recovery ends
         * @return Multiplier to apply to slowdown duration for next schedule
         */
        uint32_t ComputeSlowdownMultiplier(uint32_t queueDelayAtExit) const;

        // -----------------------------------------------------------------------
        // Member variables (original)
        // -----------------------------------------------------------------------

        Time m_target;             //!< Upper bound / default target queue delay (60 ms)
        double m_gain;             //!< GAIN value from RFC
        SlowStartType m_doSs;      //!< Permissible Slow Start State
        uint32_t m_baseHistoLen;   //!< Length of base delay history buffer
        uint32_t m_noiseFilterLen; //!< Length of current delay buffer
        Time m_lastRollover;       //!< Timestamp of last added delay
        double m_sndCwndCnt;       //!< The congestion window addition parameter
        OwdCircBuf m_baseHistory;  //!< Buffer to store the base delay
        OwdCircBuf m_noiseFilter;  //!< Buffer to store the current delay
        uint32_t m_flag;           //!< LEDBAT Flag
        uint32_t m_minCwnd;        //!< Minimum cWnd value
        double m_allowedIncrease;  //!< Allowed increase value

        // -----------------------------------------------------------------------
        // Member variables (new)
        // -----------------------------------------------------------------------

        Time m_minTarget;           //!< MOD 1: floor for adaptive target delay (default 20 ms)
        uint32_t m_slowdownMultMin; //!< MOD 2: min multiplier under high queue pressure (default 6)
        uint32_t m_slowdownMultMax; //!< MOD 2: max multiplier under low queue pressure (default 12)
    };

} // namespace ns3

#endif /* TCP_LEDBAT_H */