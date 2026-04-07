/*
 * Copyright (c) 2016 NITK Surathkal
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Ankit Deepak <adadeepak8@gmail.com>
 *
 * Modifications:
 *
 *  1. ADAPTIVE TARGET DELAY  (addresses RFC §3.4 – Low Latency Competition)
 *     The original code uses a fixed 60 ms target regardless of path RTT.
 *     On low-latency links (e.g., LAN) the bottleneck buffer drains before
 *     the queue delay ever reaches 60 ms, so LEDBAT++ behaves like plain TCP
 *     and loses its "background" property.
 *     Fix: compute an effective target = clamp(2 * base_delay, minTarget, maxTarget)
 *          where minTarget = 20 ms and maxTarget = m_target (60 ms by default).
 *     On a LAN with base_delay = 5 ms → effective target = 20 ms (floor).
 *     On a WAN with base_delay = 40 ms → effective target = 60 ms (unchanged).
 *     The user-facing "TargetDelay" attribute still controls the upper bound,
 *     so all existing simulation scripts continue to work without modification.
 *
 *  2. DYNAMIC SLOWDOWN INTERVAL  (improves RFC §4.4 – Periodic Slowdowns)
 *     The original code always schedules the next slowdown at exactly
 *     9 × slowdown_duration.  This is a good average, but:
 *       – If queues are still heavily loaded at slowdown exit, waiting 9×
 *         before the next cleanup means base-delay estimates stay inaccurate
 *         for too long, worsening latency drift and inter-LEDBAT fairness.
 *       – If queues are nearly empty, doing slowdowns at the same 9× rate
 *         wastes throughput unnecessarily.
 *     Fix: choose the multiplier based on normalised queue pressure at exit:
 *          queuePressure = queueDelay / effectiveTarget  (0 … 1+)
 *          multiplier    = round_lerp(slowdownMultMax, slowdownMultMin,
 *                                     clamp(queuePressure, 0, 1))
 *     High pressure (ratio → 1) → multiplier → slowdownMultMin (default 6)
 *     Low  pressure (ratio → 0) → multiplier → slowdownMultMax (default 12)
 *     The RFC's original behaviour (multiplier = 9) is the midpoint and is
 *     reproduced when queuePressure ≈ 0.5.  Setting both attributes to 9
 *     restores the exact original scheduling.
 */

#include "tcp-ledbat-plus-plus-modified.h"

#include "tcp-socket-state.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

    NS_LOG_COMPONENT_DEFINE("TcpLedbatPlusPlusModified");
    NS_OBJECT_ENSURE_REGISTERED(TcpLedbatPlusPlusModified);

    TypeId
    TcpLedbatPlusPlusModified::GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::TcpLedbatPlusPlusModified")
                .SetParent<TcpNewReno>()
                .AddConstructor<TcpLedbatPlusPlusModified>()
                .SetGroupName("Internet")
                // ---- original attributes (unchanged) --------------------------------
                .AddAttribute("TargetDelay",
                              "Upper bound for the adaptive target queue delay "
                              "(also used as the fixed target when AdaptiveTarget is off)",
                              TimeValue(MilliSeconds(60)),
                              MakeTimeAccessor(&TcpLedbatPlusPlusModified::m_target),
                              MakeTimeChecker())
                .AddAttribute("BaseHistoryLen",
                              "Number of Base delay samples",
                              UintegerValue(10),
                              MakeUintegerAccessor(&TcpLedbatPlusPlusModified::m_baseHistoLen),
                              MakeUintegerChecker<uint32_t>(1))
                .AddAttribute("NoiseFilterLen",
                              "Number of Current delay samples",
                              UintegerValue(4),
                              MakeUintegerAccessor(&TcpLedbatPlusPlusModified::m_noiseFilterLen),
                              MakeUintegerChecker<uint32_t>(1))
                .AddAttribute("Gain",
                              "Offset Gain",
                              DoubleValue(1.0),
                              MakeDoubleAccessor(&TcpLedbatPlusPlusModified::m_gain),
                              MakeDoubleChecker<double>(1e-6))
                .AddAttribute("SSParam",
                              "Possibility of Slow Start",
                              EnumValue(DO_SLOWSTART),
                              MakeEnumAccessor<SlowStartType>(&TcpLedbatPlusPlusModified::SetDoSs),
                              MakeEnumChecker(DO_SLOWSTART, "yes", DO_NOT_SLOWSTART, "no"))
                .AddAttribute("MinCwnd",
                              "Minimum cWnd for Ledbat",
                              UintegerValue(2),
                              MakeUintegerAccessor(&TcpLedbatPlusPlusModified::m_minCwnd),
                              MakeUintegerChecker<uint32_t>(1))
                .AddAttribute("AllowedIncrease",
                              "Allowed Increase",
                              DoubleValue(1.0),
                              MakeDoubleAccessor(&TcpLedbatPlusPlusModified::m_allowedIncrease),
                              MakeDoubleChecker<double>(1e-6))
                // ---- NEW attributes for Modification 1: Adaptive Target -------------
                .AddAttribute("MinTargetDelay",
                              "Floor for the adaptive target delay. "
                              "Effective target = clamp(2*base_delay, MinTargetDelay, TargetDelay). "
                              "Set equal to TargetDelay to disable adaptive behaviour.",
                              TimeValue(MilliSeconds(20)),
                              MakeTimeAccessor(&TcpLedbatPlusPlusModified::m_minTarget),
                              MakeTimeChecker())
                // ---- NEW attributes for Modification 2: Dynamic Slowdown Interval ---
                .AddAttribute("SlowdownMultMin",
                              "Minimum next-slowdown multiplier (used under high queue pressure). "
                              "RFC default is 9; set both Min and Max to 9 to restore original.",
                              UintegerValue(6),
                              MakeUintegerAccessor(&TcpLedbatPlusPlusModified::m_slowdownMultMin),
                              MakeUintegerChecker<uint32_t>(1))
                .AddAttribute("SlowdownMultMax",
                              "Maximum next-slowdown multiplier (used under low queue pressure). "
                              "RFC default is 9; set both Min and Max to 9 to restore original.",
                              UintegerValue(12),
                              MakeUintegerAccessor(&TcpLedbatPlusPlusModified::m_slowdownMultMax),
                              MakeUintegerChecker<uint32_t>(1));
        return tid;
    }

    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    void
    TcpLedbatPlusPlusModified::SetDoSs(SlowStartType doSS)
    {
        NS_LOG_FUNCTION(this << doSS);
        m_doSs = doSS;
        if (m_doSs)
        {
            m_flag |= LEDBAT_CAN_SS;
        }
        else
        {
            m_flag &= ~LEDBAT_CAN_SS;
        }
    }

    // ---------------------------------------------------------------------------
    // MODIFICATION 1: ComputeEffectiveTarget
    //
    // Returns the target delay to use for this RTT measurement.
    // When base delay is known:   target = clamp(2*base, m_minTarget, m_target)
    // Before any base sample:     fall back to m_target (original behaviour)
    // ---------------------------------------------------------------------------
    Time
    TcpLedbatPlusPlusModified::ComputeEffectiveTarget() const
    {
        uint32_t base = m_baseHistory.buffer.empty()
                            ? 0
                            : m_baseHistory.buffer[m_baseHistory.min];

        if (base == 0 || base == ~0U)
        {
            // No valid base delay yet – use the configured upper bound unchanged.
            return m_target;
        }

        // 2 * base_delay, clamped to [m_minTarget, m_target]
        int64_t adaptedMs = static_cast<int64_t>(2) * static_cast<int64_t>(base);
        int64_t minMs = m_minTarget.GetMilliSeconds();
        int64_t maxMs = m_target.GetMilliSeconds();

        int64_t effectiveMs = std::max(minMs, std::min(maxMs, adaptedMs));

        NS_LOG_INFO("AdaptiveTarget: base=" << base << "ms  2*base=" << adaptedMs
                                            << "ms  effective=" << effectiveMs << "ms");

        return MilliSeconds(effectiveMs);
    }

    // ---------------------------------------------------------------------------
    // MODIFICATION 2: ComputeSlowdownMultiplier
    //
    // Maps normalised queue pressure at slowdown exit → next-slowdown multiplier.
    //   pressure = queueDelayAtExit / effectiveTarget  (clamped to [0,1])
    //   multiplier = lerp(m_slowdownMultMax, m_slowdownMultMin, pressure)
    //     pressure=0 → multiplier=Max (long gap, queues were clean)
    //     pressure=1 → multiplier=Min (short gap, queues still loaded)
    //
    // Setting m_slowdownMultMin == m_slowdownMultMax == 9 exactly reproduces
    // the original RFC behaviour.
    // ---------------------------------------------------------------------------
    uint32_t
    TcpLedbatPlusPlusModified::ComputeSlowdownMultiplier(uint32_t queueDelayAtExit) const
    {
        Time effectiveTarget = ComputeEffectiveTarget();
        double targetMs = static_cast<double>(effectiveTarget.GetMilliSeconds());

        double pressure = (targetMs > 0.0)
                              ? static_cast<double>(queueDelayAtExit) / targetMs
                              : 0.0;

        // clamp to [0, 1]
        pressure = std::max(0.0, std::min(1.0, pressure));

        // linear interpolation: high pressure → small multiplier
        double mult = static_cast<double>(m_slowdownMultMax) + pressure * (static_cast<double>(m_slowdownMultMin) - static_cast<double>(m_slowdownMultMax));

        uint32_t result = static_cast<uint32_t>(std::round(mult));

        NS_LOG_INFO("DynamicSlowdown: queueDelay=" << queueDelayAtExit
                                                   << "ms  target=" << targetMs << "ms  pressure=" << pressure
                                                   << "  multiplier=" << result);

        return result;
    }

    // ---------------------------------------------------------------------------
    // Constructors / destructor
    // ---------------------------------------------------------------------------

    TcpLedbatPlusPlusModified::TcpLedbatPlusPlusModified()
        : TcpNewReno(),
          m_minTarget(MilliSeconds(20)),
          m_slowdownMultMin(6),
          m_slowdownMultMax(12)
    {
        NS_LOG_FUNCTION(this);
        InitCircBuf(m_baseHistory);
        InitCircBuf(m_noiseFilter);
        m_lastRollover = Seconds(0);
        m_sndCwndCnt = 0;
        m_flag = LEDBAT_CAN_SS;
    }

    void
    TcpLedbatPlusPlusModified::InitCircBuf(OwdCircBuf &buffer)
    {
        NS_LOG_FUNCTION(this);
        buffer.buffer.clear();
        buffer.min = 0;
    }

    TcpLedbatPlusPlusModified::TcpLedbatPlusPlusModified(const TcpLedbatPlusPlusModified &sock)
        : TcpNewReno(sock)
    {
        NS_LOG_FUNCTION(this);
        m_target = sock.m_target;
        m_gain = sock.m_gain;
        m_doSs = sock.m_doSs;
        m_baseHistoLen = sock.m_baseHistoLen;
        m_noiseFilterLen = sock.m_noiseFilterLen;
        m_baseHistory = sock.m_baseHistory;
        m_noiseFilter = sock.m_noiseFilter;
        m_lastRollover = sock.m_lastRollover;
        m_sndCwndCnt = sock.m_sndCwndCnt;
        m_flag = sock.m_flag;
        m_minCwnd = sock.m_minCwnd;
        m_allowedIncrease = sock.m_allowedIncrease;
        m_minTarget = sock.m_minTarget;             // MOD 1
        m_slowdownMultMin = sock.m_slowdownMultMin; // MOD 2
        m_slowdownMultMax = sock.m_slowdownMultMax; // MOD 2
    }

    TcpLedbatPlusPlusModified::~TcpLedbatPlusPlusModified()
    {
        NS_LOG_FUNCTION(this);
    }

    Ptr<TcpCongestionOps>
    TcpLedbatPlusPlusModified::Fork()
    {
        return CopyObject<TcpLedbatPlusPlusModified>(this);
    }

    std::string
    TcpLedbatPlusPlusModified::GetName() const
    {
        return "TcpLedbatPlusPlus";
    }

    // ---------------------------------------------------------------------------
    // Circular-buffer helpers (unchanged)
    // ---------------------------------------------------------------------------

    uint32_t
    TcpLedbatPlusPlusModified::MinCircBuf(OwdCircBuf &b)
    {
        NS_LOG_FUNCTION_NOARGS();
        if (b.buffer.empty())
        {
            return ~0U;
        }
        else
        {
            return b.buffer[b.min];
        }
    }

    uint32_t
    TcpLedbatPlusPlusModified::CurrentDelay(FilterFunction filter)
    {
        NS_LOG_FUNCTION(this);
        return filter(m_noiseFilter);
    }

    uint32_t
    TcpLedbatPlusPlusModified::BaseDelay()
    {
        NS_LOG_FUNCTION(this);
        return MinCircBuf(m_baseHistory);
    }

    // ---------------------------------------------------------------------------
    // ComputeGain
    // Uses the effective (adaptive) target instead of the fixed m_target so that
    // the GAIN formula is consistent with the rest of the algorithm.
    // ---------------------------------------------------------------------------
    double
    TcpLedbatPlusPlusModified::ComputeGain()
    {
        uint64_t base_delay = BaseDelay();

        if (base_delay == 0 || base_delay == ~0U)
        {
            return 1.0;
        }

        double base = static_cast<double>(base_delay);
        // MOD 1: use effective target so GAIN is consistent with delay comparisons
        double target = static_cast<double>(ComputeEffectiveTarget().GetMilliSeconds());

        double ratio = std::ceil(2.0 * target / base);
        double denom = std::min(16.0, ratio);

        return 1.0 / denom;
    }

    // ---------------------------------------------------------------------------
    // IncreaseWindow
    // The slowdown scheduling path is the only place changed for Modification 2.
    // Everything else is identical to the original.
    // ---------------------------------------------------------------------------
    void
    TcpLedbatPlusPlusModified::IncreaseWindow(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
    {
        NS_LOG_FUNCTION(this << tcb << segmentsAcked);
        if (tcb->m_cWnd.Get() <= tcb->m_segmentSize &&
            !(tcb->m_initialSs || tcb->m_waitingForInitialSlowdown ||
              tcb->m_inSlowdown || tcb->m_slowdownRecovery))
        {
            m_flag |= LEDBAT_CAN_SS;
        }

        uint32_t queueDelay = 0;

        if (tcb->m_initialSs && (m_flag & LEDBAT_VALID_OWD))
        {
            uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlusModified::MinCircBuf);
            uint32_t baseDelay = BaseDelay();
            queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;

            // MOD 1: compare against effective target during initial slow-start check
            double effectiveTargetMs =
                static_cast<double>(ComputeEffectiveTarget().GetMilliSeconds());

            if (static_cast<double>(queueDelay) > 0.75 * effectiveTargetMs)
            {
                NS_LOG_INFO("Exiting initial slow start due to exceeding 3/4 of effective target...");
                NS_LOG_INFO("Queue delay: " << queueDelay
                                            << " Effective target: " << effectiveTargetMs);
                tcb->m_initialSs = false;
                m_flag &= ~LEDBAT_CAN_SS;
                tcb->m_initialSsExitTime = Simulator::Now();
                tcb->m_waitingForInitialSlowdown = true;
            }
            else
            {
                m_flag |= LEDBAT_CAN_SS;
            }
        }
        else if (!(tcb->m_initialSs || tcb->m_waitingForInitialSlowdown ||
                   tcb->m_inSlowdown || tcb->m_slowdownRecovery) &&
                 (Simulator::Now() >= tcb->m_nextSlowdownTime))
        {
            // Time for a periodic slowdown
            tcb->m_inSlowdown = true;
            m_flag |= LEDBAT_CAN_SS;
            tcb->m_slowdownStartTime = Simulator::Now();
            tcb->m_ssThresh = tcb->m_cWnd;
            tcb->m_cWnd = 2 * tcb->m_segmentSize;
        }

        if (tcb->m_waitingForInitialSlowdown)
        {
            if (m_flag & LEDBAT_VALID_OWD)
            {
                uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlusModified::MinCircBuf);
                uint32_t baseDelay = BaseDelay();
                queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;
                NS_LOG_INFO("Queue delay: " << queueDelay
                                            << " Effective target: "
                                            << ComputeEffectiveTarget().GetMilliSeconds()
                                            << " Base delay: " << baseDelay);
            }
            if (Simulator::Now() - tcb->m_initialSsExitTime >= 2 * tcb->m_srtt)
            {
                tcb->m_waitingForInitialSlowdown = false;
                tcb->m_inSlowdown = true;
                tcb->m_slowdownStartTime = Simulator::Now();
                tcb->m_ssThresh = tcb->m_cWnd;
                tcb->m_cWnd = 2 * tcb->m_segmentSize;
                m_flag |= LEDBAT_CAN_SS;
                NS_LOG_INFO("Holding cwnd at 2 packets for 2 RTT...");
            }
            else
            {
                NS_LOG_INFO("Waiting 2 RTT for initial slowdown period to start...");
                m_flag &= ~LEDBAT_CAN_SS;
                CongestionAvoidance(tcb, segmentsAcked);
            }
        }
        else if (tcb->m_inSlowdown && (m_flag & LEDBAT_CAN_SS))
        {
            if (m_flag & LEDBAT_VALID_OWD)
            {
                uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlusModified::MinCircBuf);
                uint32_t baseDelay = BaseDelay();
                queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;
                NS_LOG_INFO("Queue delay: " << queueDelay
                                            << " Effective target: "
                                            << ComputeEffectiveTarget().GetMilliSeconds()
                                            << " Base delay: " << baseDelay);
            }
            if (Simulator::Now() - tcb->m_slowdownStartTime >= 2 * tcb->m_srtt)
            {
                tcb->m_inSlowdown = false;
                tcb->m_slowdownRecovery = true;
                m_flag |= LEDBAT_CAN_SS;
                NS_LOG_INFO("2 RTT over, growing cwnd to ssthresh through slow start...");
                segmentsAcked = SlowStart(tcb, segmentsAcked);
            }
            else
            {
                NS_LOG_INFO("Holding cwnd at 2 packets for 2 RTT...");
            }
        }
        else if (tcb->m_slowdownRecovery)
        {
            if (m_flag & LEDBAT_VALID_OWD)
            {
                uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlusModified::MinCircBuf);
                uint32_t baseDelay = BaseDelay();
                queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;
                NS_LOG_INFO("Queue delay: " << queueDelay
                                            << " Effective target: "
                                            << ComputeEffectiveTarget().GetMilliSeconds()
                                            << " Base delay: " << baseDelay);
            }
            if (tcb->m_cWnd >= tcb->m_ssThresh)
            {
                NS_LOG_INFO("Slowdown finished");
                tcb->m_slowdownRecovery = false;
                tcb->m_slowdownDuration = Simulator::Now() - tcb->m_slowdownStartTime;

                // ---------------------------------------------------------------
                // MOD 2: Dynamic next-slowdown interval
                //   Use queue pressure at exit to pick the multiplier, then
                //   schedule:  nextSlowdown = now + multiplier * slowdownDuration
                // ---------------------------------------------------------------
                uint32_t mult = ComputeSlowdownMultiplier(queueDelay);
                tcb->m_nextSlowdownTime = Simulator::Now() + mult * tcb->m_slowdownDuration;
                NS_LOG_INFO("Next slowdown in " << mult << " * "
                                                << tcb->m_slowdownDuration.GetMilliSeconds() << " ms");
                // ---------------------------------------------------------------

                m_flag &= ~LEDBAT_CAN_SS;
                CongestionAvoidance(tcb, segmentsAcked);
            }
            else
            {
                NS_LOG_INFO("Growing cwnd to ssthresh through slow start...");
                m_flag |= LEDBAT_CAN_SS;
                segmentsAcked = SlowStart(tcb, segmentsAcked);
            }
        }
        else if (m_doSs == DO_SLOWSTART && tcb->m_cWnd <= tcb->m_ssThresh &&
                 (m_flag & LEDBAT_CAN_SS))
        {
            if (m_flag & LEDBAT_VALID_OWD)
            {
                NS_LOG_INFO("Queue delay: " << queueDelay
                                            << " Effective target: "
                                            << ComputeEffectiveTarget().GetMilliSeconds());
            }
            segmentsAcked = SlowStart(tcb, segmentsAcked);
        }
        else
        {
            if (tcb->m_initialSs)
            {
                tcb->m_initialSs = false;
                tcb->m_initialSsExitTime = Simulator::Now();
                tcb->m_waitingForInitialSlowdown = true;
            }
            m_flag &= ~LEDBAT_CAN_SS;
            CongestionAvoidance(tcb, segmentsAcked);
        }
    }

    // ---------------------------------------------------------------------------
    // SlowStart (unchanged logic; GAIN now uses effective target via ComputeGain)
    // ---------------------------------------------------------------------------
    uint32_t
    TcpLedbatPlusPlusModified::SlowStart(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
    {
        NS_LOG_FUNCTION(this << tcb << segmentsAcked);
        if (segmentsAcked >= 1)
        {
            if ((m_flag & LEDBAT_VALID_OWD) == 0)
            {
                return TcpNewReno::SlowStart(tcb, segmentsAcked);
            }
            double gain = ComputeGain();
            tcb->m_cWnd += static_cast<uint32_t>(gain * tcb->m_segmentSize);
            NS_LOG_INFO("In SlowStart, updated to cwnd " << tcb->m_cWnd
                                                         << " ssthresh " << tcb->m_ssThresh << " gain " << gain);
            return segmentsAcked - 1;
        }
        return 0;
    }

    // ---------------------------------------------------------------------------
    // CongestionAvoidance
    // The only change is that every comparison against the target delay now uses
    // ComputeEffectiveTarget() instead of the raw m_target.
    // ---------------------------------------------------------------------------
    void
    TcpLedbatPlusPlusModified::CongestionAvoidance(Ptr<TcpSocketState> tcb, uint32_t segmentsAcked)
    {
        NS_LOG_FUNCTION(this << tcb << segmentsAcked);
        if ((m_flag & LEDBAT_VALID_OWD) == 0)
        {
            TcpNewReno::CongestionAvoidance(tcb, segmentsAcked);
            return;
        }

        uint32_t currentDelay = CurrentDelay(&TcpLedbatPlusPlusModified::MinCircBuf);
        uint32_t baseDelay = BaseDelay();
        uint32_t segmentSize = tcb->m_segmentSize;
        uint32_t cwnd = tcb->m_cWnd.Get();

        // MOD 1: use effective target for all delay comparisons
        double effectiveTargetMs =
            static_cast<double>(ComputeEffectiveTarget().GetMilliSeconds());

        uint32_t queueDelay = currentDelay > baseDelay ? currentDelay - baseDelay : 0;
        double delayRatio = (effectiveTargetMs > 0.0)
                                ? static_cast<double>(queueDelay) / effectiveTargetMs
                                : 0.0;

        double ackFactor = static_cast<double>(segmentSize) / static_cast<double>(cwnd);
        double W = static_cast<double>(cwnd) / static_cast<double>(segmentSize);
        double gain = ComputeGain();

        if (delayRatio < 1.0)
        {
            W += gain * ackFactor;
        }
        else
        {
            double md = gain - W * (delayRatio - 1.0);
            W += std::max(md * ackFactor, -W * ackFactor / 2.0);
        }

        W = std::max(W, 2.0);
        cwnd = static_cast<uint32_t>(W * segmentSize);

        NS_LOG_INFO("base_delay: " << baseDelay
                                   << " curr_delay: " << currentDelay
                                   << " queue_delay: " << queueDelay
                                   << " eff_target: " << effectiveTargetMs
                                   << " delay_ratio: " << delayRatio
                                   << " W: " << W);

        uint32_t flightSizeBeforeAck =
            tcb->m_bytesInFlight.Get() + (segmentsAcked * segmentSize);
        uint32_t maxCwnd =
            flightSizeBeforeAck + static_cast<uint32_t>(m_allowedIncrease * segmentSize);

        cwnd = std::min(cwnd, maxCwnd);
        cwnd = std::max(cwnd, m_minCwnd * segmentSize);
        tcb->m_cWnd = cwnd;

        if (tcb->m_cWnd <= tcb->m_ssThresh)
        {
            tcb->m_ssThresh = tcb->m_cWnd - 1;
        }
    }

    // ---------------------------------------------------------------------------
    // AddDelay / UpdateBaseDelay / PktsAcked  (all unchanged)
    // ---------------------------------------------------------------------------

    void
    TcpLedbatPlusPlusModified::AddDelay(OwdCircBuf &cb, uint32_t owd, uint32_t maxlen)
    {
        NS_LOG_FUNCTION(this << owd << maxlen << cb.buffer.size());
        if (cb.buffer.empty())
        {
            NS_LOG_LOGIC("First Value for queue");
            cb.buffer.push_back(owd);
            cb.min = 0;
            return;
        }
        cb.buffer.push_back(owd);
        if (cb.buffer[cb.min] > owd)
        {
            cb.min = cb.buffer.size() - 1;
        }
        if (cb.buffer.size() >= maxlen)
        {
            NS_LOG_LOGIC("Queue full" << maxlen);
            cb.buffer.erase(cb.buffer.begin());
            auto bufferStart = cb.buffer.begin();
            cb.min = std::distance(bufferStart,
                                   std::min_element(bufferStart, cb.buffer.end()));
            NS_LOG_LOGIC("Current min element" << cb.buffer[cb.min]);
        }
    }

    void
    TcpLedbatPlusPlusModified::UpdateBaseDelay(uint32_t owd)
    {
        NS_LOG_FUNCTION(this << owd);
        if (m_baseHistory.buffer.empty())
        {
            AddDelay(m_baseHistory, owd, m_baseHistoLen);
            return;
        }
        Time timestamp = Simulator::Now();
        if (timestamp - m_lastRollover > Seconds(60))
        {
            m_lastRollover = timestamp;
            AddDelay(m_baseHistory, owd, m_baseHistoLen);
        }
        else
        {
            size_t last = m_baseHistory.buffer.size() - 1;
            if (owd < m_baseHistory.buffer[last])
            {
                m_baseHistory.buffer[last] = owd;
                if (owd < m_baseHistory.buffer[m_baseHistory.min])
                {
                    m_baseHistory.min = last;
                }
            }
        }
    }

    void
    TcpLedbatPlusPlusModified::PktsAcked(Ptr<TcpSocketState> tcb,
                                 uint32_t segmentsAcked,
                                 const Time &rtt)
    {
        NS_LOG_FUNCTION(this << tcb << segmentsAcked << rtt);

        if (rtt.IsPositive() && tcb->m_rcvTimestampValue >= tcb->m_rcvTimestampEchoReply)
        {
            m_flag |= LEDBAT_VALID_OWD;
            uint32_t rttMs = rtt.GetMilliSeconds();
            AddDelay(m_noiseFilter, rttMs, m_noiseFilterLen);
            UpdateBaseDelay(rttMs);
        }
        else
        {
            m_flag &= ~LEDBAT_VALID_OWD;
        }
    }

} // namespace ns3