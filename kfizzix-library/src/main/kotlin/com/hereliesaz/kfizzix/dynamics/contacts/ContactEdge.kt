package com.hereliesaz.kfizzix.dynamics.contacts

import com.hereliesaz.kfizzix.dynamics.Body

class ContactEdge {
    var other: Body? = null
    var contact: Contact? = null
    var prev: ContactEdge? = null
    var next: ContactEdge? = null
}
