"""Feasible-path quotes for shared frontiers; this module grants no motion."""
import math
from fleet_mapping.registration import compose, inverse
from .peer_protocol import MEMBERS, finite_pose


def valid_offer(offer):
    return (isinstance(offer, dict) and isinstance(offer.get('frontier_id'), str)
            and 0 < len(offer['frontier_id']) <= 64 and finite_pose(offer.get('team_target'))
            and isinstance(offer.get('visible_unknown_m2'), (int, float))
            and math.isfinite(offer['visible_unknown_m2']) and 0 <= offer['visible_unknown_m2'] <= 1000)


def quote_frontiers(grid, pose, team_from_local, offers, team_peers):
    local = inverse(team_from_local)
    peers = [compose(local, p) for p in team_peers]
    allowed = grid.allowed(others=peers, other_clearance=.85)
    result = {}
    for offer in offers.values():
        if not valid_offer(offer):
            continue
        target = compose(local, offer['team_target'])
        route = grid.astar(pose, target, allowed)
        result[offer['frontier_id']] = float(route[0]) if route else None
    return result


def frontier_winner(offers, quotes, boots, now, maximum_age=6.):
    if not 2 <= len(boots) <= 3 or set(boots)-set(MEMBERS):
        return None
    candidates = []
    for offer in offers.values():
        if not valid_offer(offer):
            continue
        rows = quotes.get(offer['frontier_id'], {})
        costs = []
        complete = True
        for name, boot in boots.items():
            row = rows.get(name)
            if (not isinstance(row, dict) or row.get('boot_id') != boot
                    or not isinstance(row.get('sent'), (int, float))
                    or not 0 <= now-row['sent'] < maximum_age):
                complete = False
                break
            cost = row.get('cost')
            if cost is not None:
                if not isinstance(cost, (int, float)) or isinstance(cost, bool) or not math.isfinite(cost) or cost < 0:
                    complete = False
                    break
                costs.append((cost, name))
        if complete and costs:
            cost, owner = min(costs)  # Fixed robot ID resolves equal path costs.
            utility = max(.05, offer['visible_unknown_m2'])/(cost+.5)
            candidates.append((-utility, offer['frontier_id'], owner, offer))
    if not candidates:
        return None
    _, _, owner, offer = min(candidates)
    return owner, offer
